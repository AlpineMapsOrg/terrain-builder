/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
 * Copyright (C) 2026 Martin Braunsperger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#include "gis/vector_mask.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <numbers>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <cpl_error.h>
#include <fmt/format.h>
#include <gdal_priv.h>
#include <glm/glm.hpp>
#include <libassert/assert.hpp>
#include <ogrsf_frmts.h>

#include "cli_support/log.h"
#include "error/try.h"
#include "error/utils.h"
#include "geometry/geometry.h"
#include "gis/CplError.h"
#include "gis/Dataset.h"
#include "gis/srs/Kind.h"
#include "gis/srs/classify.h"
#include "gis/srs/well_known.h"
#include "mesh/convert.h"

namespace gis::vector_mask {
namespace {

// Why the polygons cannot be reprojected faithfully.
enum class InvalidReprojection {
    // Subdividing the edges does not bring them within the tolerance.
    NonConvergentEdges,
    // The reprojected polygons are not valid.
    InvalidGeometry,
};

std::string_view format_as(const InvalidReprojection error) {
    switch (error) {
    case InvalidReprojection::NonConvergentEdges:
        return "the reprojected mask edges do not converge to the tolerance";
    case InvalidReprojection::InvalidGeometry:
        return "the reprojected polygons of the mask are not valid";
    default:
        return "an unknown reprojection failure";
    }
}

alp::Result<Polygon2, InvalidMask> convert_ring(const OGRLinearRing &ring, const bool is_outer) {
    std::vector<Point2> points;
    for (int i = 0; i < ring.getNumPoints(); i++) {
        const Point2 point(ring.getX(i), ring.getY(i));
        // Repeated vertices, including the closing one, add no shape.
        if (points.empty() || point != points.back()) {
            points.push_back(point);
        }
    }
    if (points.size() > 1 && points.front() == points.back()) {
        points.pop_back();
    }
    if (points.size() < 3) {
        return alp::make_error(InvalidMask::InvalidGeometry, "checking that a ring has at least 3 distinct vertices");
    }

    Polygon2 polygon(points.begin(), points.end());
    if (!polygon.is_simple()) {
        return alp::make_error(InvalidMask::InvalidGeometry, "checking that a ring is simple");
    }
    if (is_outer != polygon.is_counterclockwise_oriented()) {
        polygon.reverse_orientation();
    }
    return polygon;
}

alp::Result<void, InvalidMask> append_polygons(const OGRGeometry &geometry, MultipolygonWithHoles2 *polygons) {
    // Maps 2.5D types to 2D.
    const OGRwkbGeometryType geometry_type = wkbFlatten(geometry.getGeometryType());
    switch (geometry_type) {
    case wkbPolygon: {
        const OGRPolygon &ogr_polygon = *geometry.toPolygon();
        if (ogr_polygon.IsEmpty()) {
            return {};
        }
        ALP_TRY(Polygon2 outer, convert_ring(*ogr_polygon.getExteriorRing(), true));
        PolygonWithHoles2 polygon(std::move(outer));
        for (int i = 0; i < ogr_polygon.getNumInteriorRings(); i++) {
            ALP_TRY(Polygon2 hole, convert_ring(*ogr_polygon.getInteriorRing(i), false));
            polygon.add_hole(std::move(hole));
        }
        polygons->add_polygon_with_holes(std::move(polygon));
        return {};
    }

    case wkbMultiPolygon:
    case wkbGeometryCollection: {
        const OGRGeometryCollection &collection = *geometry.toGeometryCollection();
        for (int i = 0; i < collection.getNumGeometries(); i++) {
            ALP_TRY(append_polygons(*collection.getGeometryRef(i), polygons));
        }
        return {};
    }

    default:
        return alp::make_error(InvalidMask::UnsupportedGeometry, fmt::format("reading a geometry of type {}", OGRGeometryTypeToName(geometry_type)));
    }
}

size_t vertex_count(const MultipolygonWithHoles2 &polygons) {
    size_t count = 0;
    for (const auto &polygon : polygons.polygons_with_holes()) {
        count += polygon.outer_boundary().size();
        for (const auto &hole : polygon.holes()) {
            count += hole.size();
        }
    }
    return count;
}

// Tolerances in SRS units on a coarse grid over the given bounds.
// Each cell holds the tolerance at the corner where the SRS stretches the most in any direction.
class ToleranceGrid {
public:
    static alp::Result<ToleranceGrid, UnsupportedToleranceSrs> make(const srs::System &srs, const CGAL::Bbox_2 &bounds, double tolerance_m);

    double at(const glm::dvec2 &point) const {
        const glm::dvec2 cell = glm::clamp(glm::floor((point - this->m_origin) / this->m_cell_size), glm::dvec2(0), glm::dvec2(this->m_cell_count - 1));
        return this->m_tolerances[static_cast<size_t>(cell.y) * this->m_cell_count.x + static_cast<size_t>(cell.x)];
    }

private:
    glm::dvec2 m_origin = {};
    glm::dvec2 m_cell_size = {};
    glm::ivec2 m_cell_count = {};
    std::vector<double> m_tolerances;
};

alp::Result<ToleranceGrid, UnsupportedToleranceSrs> ToleranceGrid::make(const srs::System &srs, const CGAL::Bbox_2 &bounds, const double tolerance_m) {
    // About one degree, or 100 km.
    double nominal_cell_size = 0;
    if (srs.kind() == srs::Kind::Geographic) {
        nominal_cell_size = std::numbers::pi / 180 / srs.as_gdal().GetAngularUnits();
    } else if (srs.kind() == srs::Kind::Projected) {
        nominal_cell_size = 100'000 / srs.as_gdal().GetLinearUnits();
    }
    if (!(nominal_cell_size > 0)) {
        return alp::make_error(UnsupportedToleranceSrs{}, fmt::format("expressing the mask tolerance in {}", srs));
    }

    ToleranceGrid grid;
    grid.m_origin = {bounds.xmin(), bounds.ymin()};
    const glm::dvec2 extent = {bounds.xmax() - bounds.xmin(), bounds.ymax() - bounds.ymin()};
    for (int32_t axis = 0; axis < 2; axis++) {
        grid.m_cell_count[axis] = std::clamp(static_cast<int32_t>(std::ceil(extent[axis] / nominal_cell_size)), 1, 1024);
        grid.m_cell_size[axis] = extent[axis] > 0 ? extent[axis] / grid.m_cell_count[axis] : nominal_cell_size;
    }

    // Per corner, the corner itself and one step along x and y.
    const glm::ivec2 corner_count = grid.m_cell_count + 1;
    const glm::dvec2 centre = grid.m_origin + extent / 2.0;
    const double step = nominal_cell_size * 1e-3;
    std::vector<glm::dvec3> samples;
    samples.reserve(3 * static_cast<size_t>(corner_count.x) * corner_count.y);
    for (int32_t y = 0; y < corner_count.y; y++) {
        for (int32_t x = 0; x < corner_count.x; x++) {
            const glm::dvec2 corner = grid.m_origin + glm::dvec2(x, y) * grid.m_cell_size;
            // Step towards the centre, to stay within the SRS's domain, e.g., at the poles.
            const glm::dvec2 offset = {corner.x < centre.x ? step : -step, corner.y < centre.y ? step : -step};
            samples.emplace_back(corner, 0.0);
            samples.emplace_back(corner.x + offset.x, corner.y, 0.0);
            samples.emplace_back(corner.x, corner.y + offset.y, 0.0);
        }
    }
    // Metres are measured in ECEF, so a system that cannot be mapped there cannot express the tolerance.
    auto transform_result = alp::map_error<srs::UnsupportedTransform>(srs::get_transform(srs, srs::ecef()), UnsupportedToleranceSrs{});
    ALP_TRY_CONTEXT(const srs::Transform transform, std::move(transform_result), "measuring metres per unit of the mask's system");
    auto samples_result = alp::map_error<UntransformablePoint>(transform.apply_inplace(&samples), UnsupportedToleranceSrs{});
    ALP_TRY_CONTEXT(std::move(samples_result), "measuring metres per unit of the mask's system");

    std::vector<double> stretches(samples.size() / 3);
    for (size_t i = 0; i < stretches.size(); i++) {
        const glm::dvec3 along_x = (samples[3 * i + 1] - samples[3 * i]) / step;
        const glm::dvec3 along_y = (samples[3 * i + 2] - samples[3 * i]) / step;
        // Largest singular value of the Jacobian: the most metres per unit in any direction.
        const double xx = glm::dot(along_x, along_x);
        const double yy = glm::dot(along_y, along_y);
        const double xy = glm::dot(along_x, along_y);
        stretches[i] = std::sqrt((xx + yy) / 2 + std::sqrt((xx - yy) * (xx - yy) / 4 + xy * xy));
    }

    grid.m_tolerances.reserve(static_cast<size_t>(grid.m_cell_count.x) * grid.m_cell_count.y);
    for (int32_t y = 0; y < grid.m_cell_count.y; y++) {
        for (int32_t x = 0; x < grid.m_cell_count.x; x++) {
            const auto stretch = [&](const int32_t corner_x, const int32_t corner_y) { return stretches[static_cast<size_t>(corner_y) * corner_count.x + corner_x]; };
            const double max_stretch = std::max({stretch(x, y), stretch(x + 1, y), stretch(x, y + 1), stretch(x + 1, y + 1)});
            grid.m_tolerances.push_back(tolerance_m / max_stretch);
        }
    }
    return grid;
}

using SimplificationKernel = cgal::kernel::epick::Kernel;
using SimplificationTriangulation = CGAL::Constrained_triangulation_plus_2<CGAL::Constrained_Delaunay_triangulation_2<SimplificationKernel,
    CGAL::Triangulation_data_structure_2<CGAL::Polyline_simplification_2::Vertex_base_2<SimplificationKernel>,
        CGAL::Constrained_triangulation_face_base_2<SimplificationKernel>>,
    CGAL::Exact_predicates_tag>>;

// Cost of removing a vertex: the largest deviation of the original points from the new edge, relative to their local tolerance.
struct RelativeDeviationCost {
    const ToleranceGrid *tolerances;

    template <class Triangulation>
    std::optional<typename Triangulation::Geom_traits::FT> operator()(const CGAL::Constrained_triangulation_plus_2<Triangulation> &,
        typename CGAL::Constrained_triangulation_plus_2<Triangulation>::Vertices_in_constraint_iterator vertex) const {
        using PointsInConstraintIterator = typename CGAL::Constrained_triangulation_plus_2<Triangulation>::Points_in_constraint_iterator;
        const auto previous = std::prev(vertex);
        const auto next = std::next(vertex);
        const radix::geometry::Edge<2, double> edge = {convert::to_glm_point((*previous)->point()), convert::to_glm_point((*next)->point())};

        double cost = 0;
        PointsInConstraintIterator point(previous);
        const PointsInConstraintIterator end(next);
        for (point++; point != end; point++) {
            const glm::dvec2 original = convert::to_glm_point(*point);
            cost = std::max(cost, geometry::distance_to_segment(original, edge) / this->tolerances->at(original));
        }
        return cost;
    }
};

SimplificationTriangulation::Constraint_id insert_ring(SimplificationTriangulation *triangulation, const Polygon2 &ring) {
    std::vector<SimplificationKernel::Point_2> points;
    points.reserve(ring.size());
    for (const Point2 &point : ring) {
        points.push_back(convert::to_cgal_point<SimplificationKernel>(convert::to_glm_point(point)));
    }
    return triangulation->insert_constraint(points.begin(), points.end(), true);
}

Polygon2 extract_ring(const SimplificationTriangulation &triangulation, const SimplificationTriangulation::Constraint_id id) {
    std::vector<Point2> points;
    for (auto vertex = triangulation.vertices_in_constraint_begin(id); vertex != triangulation.vertices_in_constraint_end(id); vertex++) {
        points.push_back(convert::to_cgal_point<Kernel>(convert::to_glm_point((*vertex)->point())));
    }
    // Closed constraints repeat their first vertex.
    points.pop_back();
    return Polygon2(points.begin(), points.end());
}

// A ring during reprojection, with its vertices in both SRSs.
struct ReprojectedRing {
    std::vector<glm::dvec2> source;
    std::vector<glm::dvec2> target;
    // Whether the edge from each vertex to the next is within the tolerance.
    std::vector<bool> accurate;
};

// Splits edges whose midpoint deviates by more than the tolerance until all edges are accurate.
// Each round reprojects all new midpoints at once.
alp::Result<void, alp::Error<InvalidReprojection, UntransformablePoint>> subdivide(const srs::Transform &transform, const ToleranceGrid &tolerances, std::vector<ReprojectedRing> *rings) {
    constexpr int32_t max_rounds = 32;
    for (int32_t round = 0;; round++) {
        std::vector<glm::dvec2> midpoints;
        for (const ReprojectedRing &ring : *rings) {
            for (size_t i = 0; i < ring.source.size(); i++) {
                if (!ring.accurate[i]) {
                    midpoints.push_back((ring.source[i] + ring.source[(i + 1) % ring.source.size()]) / 2.0);
                }
            }
        }
        if (midpoints.empty()) {
            return {};
        }
        if (round == max_rounds) {
            return alp::make_error(InvalidReprojection::NonConvergentEdges);
        }
        std::vector<glm::dvec2> reprojected_midpoints = midpoints;
        ALP_TRY_CONTEXT(transform.apply_inplace(&reprojected_midpoints), "reprojecting the midpoints of the mask edges");

        size_t midpoint = 0;
        for (ReprojectedRing &ring : *rings) {
            ReprojectedRing refined;
            for (size_t i = 0; i < ring.source.size(); i++) {
                refined.source.push_back(ring.source[i]);
                refined.target.push_back(ring.target[i]);
                if (ring.accurate[i]) {
                    refined.accurate.push_back(true);
                    continue;
                }
                const glm::dvec2 &target_midpoint = reprojected_midpoints[midpoint];
                const radix::geometry::Edge<2, double> edge = {ring.target[i], ring.target[(i + 1) % ring.target.size()]};
                if (geometry::distance_to_segment(target_midpoint, edge) <= tolerances.at(target_midpoint)) {
                    refined.accurate.push_back(true);
                } else {
                    refined.accurate.push_back(false);
                    refined.source.push_back(midpoints[midpoint]);
                    refined.target.push_back(target_midpoint);
                    refined.accurate.push_back(false);
                }
                midpoint++;
            }
            ring = std::move(refined);
        }
    }
}

alp::Result<Polygon2, InvalidReprojection> to_polygon(const ReprojectedRing &ring, const bool is_outer) {
    Polygon2 polygon;
    for (const glm::dvec2 &point : ring.target) {
        polygon.push_back(convert::to_cgal_point<Kernel>(point));
    }
    if (!polygon.is_simple()) {
        return alp::make_error(InvalidReprojection::InvalidGeometry, "checking that a reprojected ring is simple");
    }
    if (is_outer != polygon.is_counterclockwise_oriented()) {
        polygon.reverse_orientation();
    }
    return polygon;
}

} // namespace

alp::Result<ReferencedPolygonMask, LoadMaskError> load(Dataset &dataset) {
    const auto context = alp::make_context("loading mask {}", dataset.name());
    GDALDataset *gdal_dataset = dataset.as_gdal();

    auto srs_result = dataset.srs();
    if (!srs_result) {
        LOG_WARN("Mask {} does not reference an SRS, assuming WGS84", dataset.name());
    }
    const srs::System srs = srs_result ? *std::move(srs_result) : srs::wgs84();
    // OGR gives feature coordinates in the data axis order of the layer's system.
    if (!srs::is_east_north_axis_order(srs)) {
        return alp::make_error(srs::UnsupportedAxisOrder{}, context);
    }
    // All layers are read in the same SRS.
    for (int i = 0; i < gdal_dataset->GetLayerCount(); i++) {
        OGRLayer *layer = gdal_dataset->GetLayer(i);
        const OGRSpatialReference *layer_srs = layer != nullptr ? layer->GetSpatialRef() : nullptr;
        if (layer_srs != nullptr && !srs.as_gdal().IsSame(layer_srs)) {
            return alp::make_error(InvalidMask::AmbiguousSystem, context);
        }
    }

    MultipolygonWithHoles2 polygons;
    CPLErrorReset();
    for (auto &&feature_layer_pair : gdal_dataset->GetFeatures()) {
        const OGRFeature &feature = *feature_layer_pair.feature;
        const auto feature_context = alp::make_context("loading feature {} of mask {}", feature.GetFID(), dataset.name());
        const OGRGeometry *geometry = feature.GetGeometryRef();
        if (geometry == nullptr) {
            return alp::make_error(InvalidMask::InvalidGeometry, feature_context);
        }
        ALP_TRY_CONTEXT(append_polygons(*geometry, &polygons), feature_context);
    }
    if (CPLGetLastErrorType() >= CE_Failure) {
        return alp::make_error(UnreadableMask{}, context, CplError::get_last_error());
    }
    if (polygons.is_empty()) {
        return alp::make_error(InvalidMask::MissingGeometry, context);
    }

    return ReferencedPolygonMask{.polygons = std::move(polygons), .srs = srs};
}

alp::Result<ReferencedPolygonMask, TransformMaskError> transform(const ReferencedPolygonMask &mask, const srs::System &target_srs, const double tolerance_m) {
    ASSERT(tolerance_m > 0);
    if (mask.srs.is_same(target_srs)) {
        return mask;
    }
    // Every failure to reproject means the same to a caller, so the details stay in the source.
    auto transformation_result = alp::map_error<srs::UnsupportedTransform>(srs::get_transform(mask.srs, target_srs), UnreprojectableMask{});
    ALP_TRY_CONTEXT(const srs::Transform transformation, std::move(transformation_result), "reprojecting the mask");

    std::vector<ReprojectedRing> rings;
    std::vector<glm::dvec2> vertices;
    const auto add_ring = [&](const Polygon2 &polygon) {
        ReprojectedRing &ring = rings.emplace_back();
        for (const Point2 &point : polygon) {
            ring.source.push_back(convert::to_glm_point(point));
        }
        ring.accurate.resize(ring.source.size(), false);
        vertices.insert(vertices.end(), ring.source.begin(), ring.source.end());
    };
    for (const auto &polygon : mask.polygons.polygons_with_holes()) {
        add_ring(polygon.outer_boundary());
        for (const auto &hole : polygon.holes()) {
            add_ring(hole);
        }
    }
    auto vertices_result = alp::map_error<UntransformablePoint>(transformation.apply_inplace(&vertices), UnreprojectableMask{});
    ALP_TRY_CONTEXT(std::move(vertices_result), "reprojecting the mask vertices");
    CGAL::Bbox_2 target_bounds;
    auto reprojected_vertex = vertices.cbegin();
    for (ReprojectedRing &ring : rings) {
        ring.target.assign(reprojected_vertex, reprojected_vertex + static_cast<std::ptrdiff_t>(ring.source.size()));
        reprojected_vertex += static_cast<std::ptrdiff_t>(ring.source.size());
        for (const glm::dvec2 &point : ring.target) {
            target_bounds += CGAL::Bbox_2(point.x, point.y, point.x, point.y);
        }
    }

    ALP_TRY_CONTEXT(const ToleranceGrid tolerances, ToleranceGrid::make(target_srs, target_bounds, tolerance_m), "reprojecting the mask");
    auto subdivide_result = alp::map_error<InvalidReprojection, UntransformablePoint>(subdivide(transformation, tolerances, &rings), UnreprojectableMask{});
    ALP_TRY_CONTEXT(std::move(subdivide_result), "reprojecting the mask");

    MultipolygonWithHoles2 result;
    auto ring = rings.cbegin();
    for (const auto &polygon : mask.polygons.polygons_with_holes()) {
        auto outer_result = alp::map_error<InvalidReprojection>(to_polygon(*ring++, true), UnreprojectableMask{});
        ALP_TRY_CONTEXT(Polygon2 outer, std::move(outer_result), "reprojecting the mask");
        PolygonWithHoles2 reprojected(std::move(outer));
        for (size_t i = 0; i < polygon.number_of_holes(); i++) {
            auto hole_result = alp::map_error<InvalidReprojection>(to_polygon(*ring++, false), UnreprojectableMask{});
            ALP_TRY_CONTEXT(Polygon2 hole, std::move(hole_result), "reprojecting the mask");
            reprojected.add_hole(std::move(hole));
        }
        result.add_polygon_with_holes(std::move(reprojected));
    }

    LOG_DEBUG("Reprojected mask from {} to {} vertices", vertex_count(mask.polygons), vertex_count(result));
    return ReferencedPolygonMask{.polygons = std::move(result), .srs = target_srs};
}

alp::Result<ReferencedPolygonMask, SimplifyMaskError> simplify(const ReferencedPolygonMask &mask, const double tolerance_m) {
    ASSERT(tolerance_m > 0);
    ALP_TRY_CONTEXT(const ToleranceGrid tolerances, ToleranceGrid::make(mask.srs, mask.polygons.bbox(), tolerance_m), "simplifying the mask");

    // All rings are constraints of one triangulation, which keeps them from crossing each other.
    SimplificationTriangulation triangulation;
    std::vector<std::vector<SimplificationTriangulation::Constraint_id>> ring_ids;
    for (const auto &polygon : mask.polygons.polygons_with_holes()) {
        auto &ids = ring_ids.emplace_back();
        ids.push_back(insert_ring(&triangulation, polygon.outer_boundary()));
        for (const auto &hole : polygon.holes()) {
            ids.push_back(insert_ring(&triangulation, hole));
        }
    }
    CGAL::Polyline_simplification_2::simplify(triangulation, RelativeDeviationCost{&tolerances}, CGAL::Polyline_simplification_2::Stop_above_cost_threshold(1));

    MultipolygonWithHoles2 result;
    for (const auto &ids : ring_ids) {
        PolygonWithHoles2 polygon(extract_ring(triangulation, ids.front()));
        for (auto id = std::next(ids.begin()); id != ids.end(); id++) {
            polygon.add_hole(extract_ring(triangulation, *id));
        }
        result.add_polygon_with_holes(std::move(polygon));
    }

    LOG_DEBUG("Simplified mask with {} m tolerance from {} to {} vertices", tolerance_m, vertex_count(mask.polygons), vertex_count(result));
    return ReferencedPolygonMask{.polygons = std::move(result), .srs = mask.srs};
}

} // namespace gis::vector_mask
