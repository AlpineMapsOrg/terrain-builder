/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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
#include <iterator>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <fmt/core.h>
#include <gdal_priv.h>
#include <glm/glm.hpp>
#include <libassert/assert.hpp>
#include <ogrsf_frmts.h>

#include "cli_support/log.h"
#include "geometry/geometry.h"
#include "gis/Dataset.h"
#include "gis/srs.h"
#include "mesh/convert.h"

namespace gis::vector_mask {
namespace {

    Expected<Polygon2> convert_ring(const OGRLinearRing& ring, const bool is_outer)
    {
        std::vector<Point2> points;
        for (int i = 0; i < ring.getNumPoints(); ++i) {
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
            return Error::fail(Error::Code::CorruptData, "mask ring has fewer than 3 distinct vertices");
        }

        Polygon2 polygon(points.begin(), points.end());
        if (!polygon.is_simple()) {
            return Error::fail(Error::Code::CorruptData, "mask ring is not simple");
        }
        if (is_outer != polygon.is_counterclockwise_oriented()) {
            polygon.reverse_orientation();
        }
        return polygon;
    }

    Expected<void> append_polygons(const OGRGeometry& geometry, MultipolygonWithHoles2* polygons)
    {
        const OGRwkbGeometryType geometry_type = wkbFlatten(geometry.getGeometryType()); // map 2.5d to 2d
        switch (geometry_type) {
        case wkbPolygon: {
            const OGRPolygon& ogr_polygon = *geometry.toPolygon();
            if (ogr_polygon.IsEmpty()) {
                return {};
            }
            auto outer = convert_ring(*ogr_polygon.getExteriorRing(), true);
            if (!outer) {
                return Error::propagate(std::move(outer));
            }
            PolygonWithHoles2 polygon(std::move(*outer));
            for (int i = 0; i < ogr_polygon.getNumInteriorRings(); ++i) {
                auto hole = convert_ring(*ogr_polygon.getInteriorRing(i), false);
                if (!hole) {
                    return Error::propagate(std::move(hole));
                }
                polygon.add_hole(std::move(*hole));
            }
            polygons->add_polygon_with_holes(std::move(polygon));
            return {};
        }

        case wkbMultiPolygon:
        case wkbGeometryCollection: {
            const OGRGeometryCollection& collection = *geometry.toGeometryCollection();
            for (int i = 0; i < collection.getNumGeometries(); ++i) {
                if (auto appended = append_polygons(*collection.getGeometryRef(i), polygons); !appended) {
                    return appended;
                }
            }
            return {};
        }

        default:
            return Error::fail(Error::Code::Unsupported, fmt::format("mask geometry type {}", OGRGeometryTypeToName(geometry_type)));
        }
    }

    std::size_t vertex_count(const MultipolygonWithHoles2& polygons)
    {
        std::size_t count = 0;
        for (const auto& polygon : polygons.polygons_with_holes()) {
            count += polygon.outer_boundary().size();
            for (const auto& hole : polygon.holes()) {
                count += hole.size();
            }
        }
        return count;
    }

    // Tolerances in SRS units on a coarse grid over the given bounds. Each cell holds the
    // tolerance at the corner where the SRS stretches the most in any direction.
    class ToleranceGrid {
    public:
        static Expected<ToleranceGrid> make(const OGRSpatialReference& srs, const CGAL::Bbox_2& bounds, double tolerance_m);

        double at(const glm::dvec2& point) const
        {
            const glm::dvec2 cell = glm::clamp(glm::floor((point - m_origin) / m_cell_size), glm::dvec2(0), glm::dvec2(m_cell_count - 1));
            return m_tolerances[std::size_t(cell.y) * m_cell_count.x + std::size_t(cell.x)];
        }

    private:
        glm::dvec2 m_origin = {};
        glm::dvec2 m_cell_size = {};
        glm::ivec2 m_cell_count = {};
        std::vector<double> m_tolerances;
    };

    Expected<ToleranceGrid> ToleranceGrid::make(const OGRSpatialReference& srs, const CGAL::Bbox_2& bounds, const double tolerance_m)
    {
        // About one degree, or 100 km.
        double nominal_cell_size = 0;
        if (srs.IsGeographic()) {
            nominal_cell_size = std::numbers::pi / 180 / srs.GetAngularUnits();
        } else if (srs.IsProjected()) {
            nominal_cell_size = 100'000 / srs.GetLinearUnits();
        }
        if (!(nominal_cell_size > 0)) {
            return Error::fail(Error::Code::Unsupported, "express the mask tolerance in " + srs::friendly_name(srs));
        }

        ToleranceGrid grid;
        grid.m_origin = { bounds.xmin(), bounds.ymin() };
        const glm::dvec2 extent = { bounds.xmax() - bounds.xmin(), bounds.ymax() - bounds.ymin() };
        for (int axis = 0; axis < 2; ++axis) {
            grid.m_cell_count[axis] = std::clamp(int(std::ceil(extent[axis] / nominal_cell_size)), 1, 1024);
            grid.m_cell_size[axis] = extent[axis] > 0 ? extent[axis] / grid.m_cell_count[axis] : nominal_cell_size;
        }

        // Per corner, the corner itself and one step along x and y.
        const glm::ivec2 corner_count = grid.m_cell_count + 1;
        const glm::dvec2 centre = grid.m_origin + extent / 2.0;
        const double step = nominal_cell_size * 1e-3;
        std::vector<glm::dvec3> samples;
        samples.reserve(3 * std::size_t(corner_count.x) * corner_count.y);
        for (int y = 0; y < corner_count.y; ++y) {
            for (int x = 0; x < corner_count.x; ++x) {
                const glm::dvec2 corner = grid.m_origin + glm::dvec2(x, y) * grid.m_cell_size;
                // Step towards the centre, to stay within the SRS's domain, e.g., at the poles.
                const glm::dvec2 offset = { corner.x < centre.x ? step : -step, corner.y < centre.y ? step : -step };
                samples.emplace_back(corner, 0.0);
                samples.emplace_back(corner.x + offset.x, corner.y, 0.0);
                samples.emplace_back(corner.x, corner.y + offset.y, 0.0);
            }
        }
        auto transformation = srs::transformation(srs, srs::ecef());
        if (!transformation) {
            return Error::propagate(std::move(transformation));
        }
        if (auto transformed = srs::transform_points_inplace(transformation->get(), samples); !transformed) {
            return Error::propagate(std::move(transformed), "measure metres per SRS unit");
        }

        std::vector<double> stretches(samples.size() / 3);
        for (std::size_t i = 0; i < stretches.size(); ++i) {
            const glm::dvec3 along_x = (samples[3 * i + 1] - samples[3 * i]) / step;
            const glm::dvec3 along_y = (samples[3 * i + 2] - samples[3 * i]) / step;
            // Largest singular value of the Jacobian: the most metres per unit in any direction.
            const double xx = glm::dot(along_x, along_x);
            const double yy = glm::dot(along_y, along_y);
            const double xy = glm::dot(along_x, along_y);
            stretches[i] = std::sqrt((xx + yy) / 2 + std::sqrt((xx - yy) * (xx - yy) / 4 + xy * xy));
        }

        grid.m_tolerances.reserve(std::size_t(grid.m_cell_count.x) * grid.m_cell_count.y);
        for (int y = 0; y < grid.m_cell_count.y; ++y) {
            for (int x = 0; x < grid.m_cell_count.x; ++x) {
                const auto stretch = [&](int corner_x, int corner_y) { return stretches[std::size_t(corner_y) * corner_count.x + corner_x]; };
                const double max_stretch = std::max({ stretch(x, y), stretch(x + 1, y), stretch(x, y + 1), stretch(x + 1, y + 1) });
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

    // Cost of removing a vertex: the largest deviation of the original points from the new edge,
    // relative to their local tolerance.
    struct RelativeDeviationCost {
        const ToleranceGrid* tolerances;

        template <class Triangulation>
        std::optional<typename Triangulation::Geom_traits::FT> operator()(const CGAL::Constrained_triangulation_plus_2<Triangulation>&,
            typename CGAL::Constrained_triangulation_plus_2<Triangulation>::Vertices_in_constraint_iterator vertex) const
        {
            using PointsInConstraintIterator = typename CGAL::Constrained_triangulation_plus_2<Triangulation>::Points_in_constraint_iterator;
            const auto previous = std::prev(vertex);
            const auto next = std::next(vertex);
            const radix::geometry::Edge<2, double> edge = { convert::to_glm_point((*previous)->point()), convert::to_glm_point((*next)->point()) };

            double cost = 0;
            PointsInConstraintIterator point(previous);
            const PointsInConstraintIterator end(next);
            for (++point; point != end; ++point) {
                const glm::dvec2 original = convert::to_glm_point(*point);
                cost = std::max(cost, geometry::distance_to_segment(original, edge) / tolerances->at(original));
            }
            return cost;
        }
    };

    SimplificationTriangulation::Constraint_id insert_ring(SimplificationTriangulation* triangulation, const Polygon2& ring)
    {
        std::vector<SimplificationKernel::Point_2> points;
        points.reserve(ring.size());
        for (const Point2& point : ring) {
            points.push_back(convert::to_cgal_point<SimplificationKernel>(convert::to_glm_point(point)));
        }
        return triangulation->insert_constraint(points.begin(), points.end(), true);
    }

    Polygon2 extract_ring(const SimplificationTriangulation& triangulation, const SimplificationTriangulation::Constraint_id id)
    {
        std::vector<Point2> points;
        for (auto vertex = triangulation.vertices_in_constraint_begin(id); vertex != triangulation.vertices_in_constraint_end(id); ++vertex) {
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

    Expected<std::vector<glm::dvec2>> reproject(OGRCoordinateTransformation* transformation, std::vector<glm::dvec2> points)
    {
        if (auto transformed = srs::transform_points_inplace(transformation, points); !transformed) {
            return Error::propagate(std::move(transformed), "reproject mask vertices");
        }
        return points;
    }

    // Splits edges whose midpoint deviates by more than the tolerance until all edges are accurate.
    // Each round reprojects all new midpoints at once.
    Expected<void> subdivide(OGRCoordinateTransformation* transformation, const ToleranceGrid& tolerances, std::vector<ReprojectedRing>* rings)
    {
        constexpr int max_rounds = 32;
        for (int round = 0;; ++round) {
            std::vector<glm::dvec2> midpoints;
            for (const ReprojectedRing& ring : *rings) {
                for (std::size_t i = 0; i < ring.source.size(); ++i) {
                    if (!ring.accurate[i]) {
                        midpoints.push_back((ring.source[i] + ring.source[(i + 1) % ring.source.size()]) / 2.0);
                    }
                }
            }
            if (midpoints.empty()) {
                return {};
            }
            if (round == max_rounds) {
                return Error::fail(Error::Code::InvalidInput, "reprojected mask edges do not converge to the tolerance");
            }
            auto reprojected_midpoints = reproject(transformation, midpoints);
            if (!reprojected_midpoints) {
                return Error::propagate(std::move(reprojected_midpoints));
            }

            std::size_t midpoint = 0;
            for (ReprojectedRing& ring : *rings) {
                ReprojectedRing refined;
                for (std::size_t i = 0; i < ring.source.size(); ++i) {
                    refined.source.push_back(ring.source[i]);
                    refined.target.push_back(ring.target[i]);
                    if (ring.accurate[i]) {
                        refined.accurate.push_back(true);
                        continue;
                    }
                    const glm::dvec2& target_midpoint = (*reprojected_midpoints)[midpoint];
                    const radix::geometry::Edge<2, double> edge = { ring.target[i], ring.target[(i + 1) % ring.target.size()] };
                    if (geometry::distance_to_segment(target_midpoint, edge) <= tolerances.at(target_midpoint)) {
                        refined.accurate.push_back(true);
                    } else {
                        refined.accurate.push_back(false);
                        refined.source.push_back(midpoints[midpoint]);
                        refined.target.push_back(target_midpoint);
                        refined.accurate.push_back(false);
                    }
                    ++midpoint;
                }
                ring = std::move(refined);
            }
        }
    }

    Expected<Polygon2> to_polygon(const ReprojectedRing& ring, const bool is_outer)
    {
        Polygon2 polygon;
        for (const glm::dvec2& point : ring.target) {
            polygon.push_back(convert::to_cgal_point<Kernel>(point));
        }
        if (!polygon.is_simple()) {
            return Error::fail(Error::Code::CorruptData, "reprojected mask ring is not simple");
        }
        if (is_outer != polygon.is_counterclockwise_oriented()) {
            polygon.reverse_orientation();
        }
        return polygon;
    }

} // namespace

Expected<ReferencedPolygonMask> load(Dataset& dataset)
{
    GDALDataset* gdal_dataset = dataset.gdalDataset();

    OGRSpatialReference srs;
    if (auto dataset_srs = dataset.srs()) {
        srs = std::move(*dataset_srs);
    } else {
        LOG_WARN("Mask {} does not reference an SRS, assuming WGS84", dataset.name());
        srs = srs::wgs84();
    }
    // All layers are read in the same SRS.
    for (int i = 0; i < gdal_dataset->GetLayerCount(); ++i) {
        OGRLayer* layer = gdal_dataset->GetLayer(i);
        if (const OGRSpatialReference* layer_srs = layer ? layer->GetSpatialRef() : nullptr) {
            OGRSpatialReference normalized = *layer_srs;
            normalized.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
            if (!srs.IsSame(&normalized)) {
                return Error::fail(Error::Code::Unsupported, "mask " + dataset.name() + " has layers with different SRSs");
            }
        }
    }

    MultipolygonWithHoles2 polygons;
    CPLErrorReset();
    for (auto&& feature_layer_pair : gdal_dataset->GetFeatures()) {
        const OGRFeature& feature = *feature_layer_pair.feature;
        const OGRGeometry* geometry = feature.GetGeometryRef();
        if (!geometry) {
            return Error::fail(Error::Code::CorruptData, fmt::format("mask feature {} has no geometry", feature.GetFID()));
        }
        if (auto appended = append_polygons(*geometry, &polygons); !appended) {
            return Error::propagate(std::move(appended), fmt::format("load mask feature {}", feature.GetFID()));
        }
    }
    if (CPLGetLastErrorType() >= CE_Failure) {
        return Error::fail(Error::Code::Io, "read features of mask " + dataset.name() + ": " + CPLGetLastErrorMsg());
    }
    if (polygons.is_empty()) {
        return Error::fail(Error::Code::InvalidInput, "mask " + dataset.name() + " contains no polygons");
    }

    return ReferencedPolygonMask { .polygons = std::move(polygons), .srs = std::move(srs) };
}

Expected<ReferencedPolygonMask> transform(const Expected<ReferencedPolygonMask>& polygons, const OGRSpatialReference& target_srs, const double tolerance_m)
{
    ASSERT(tolerance_m > 0);
    if (!polygons) {
        return std::unexpected(polygons.error());
    }
    // Coordinates are always in the traditional GIS order, as with load.
    OGRSpatialReference normalized_target_srs = target_srs;
    normalized_target_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    if (polygons->srs.IsSame(&normalized_target_srs)) {
        return *polygons;
    }
    auto transformation = srs::transformation(polygons->srs, normalized_target_srs);
    if (!transformation) {
        return Error::propagate(std::move(transformation), "reproject mask");
    }

    std::vector<ReprojectedRing> rings;
    std::vector<glm::dvec2> vertices;
    const auto add_ring = [&](const Polygon2& polygon) {
        ReprojectedRing& ring = rings.emplace_back();
        for (const Point2& point : polygon) {
            ring.source.push_back(convert::to_glm_point(point));
        }
        ring.accurate.resize(ring.source.size(), false);
        vertices.insert(vertices.end(), ring.source.begin(), ring.source.end());
    };
    for (const auto& polygon : polygons->polygons.polygons_with_holes()) {
        add_ring(polygon.outer_boundary());
        for (const auto& hole : polygon.holes()) {
            add_ring(hole);
        }
    }
    auto reprojected_vertices = reproject(transformation->get(), std::move(vertices));
    if (!reprojected_vertices) {
        return Error::propagate(std::move(reprojected_vertices));
    }
    CGAL::Bbox_2 target_bounds;
    auto reprojected_vertex = reprojected_vertices->begin();
    for (ReprojectedRing& ring : rings) {
        ring.target.assign(reprojected_vertex, reprojected_vertex + std::ptrdiff_t(ring.source.size()));
        reprojected_vertex += std::ptrdiff_t(ring.source.size());
        for (const glm::dvec2& point : ring.target) {
            target_bounds += CGAL::Bbox_2(point.x, point.y, point.x, point.y);
        }
    }

    auto tolerances = ToleranceGrid::make(normalized_target_srs, target_bounds, tolerance_m);
    if (!tolerances) {
        return Error::propagate(std::move(tolerances), "reproject mask");
    }
    if (auto subdivided = subdivide(transformation->get(), *tolerances, &rings); !subdivided) {
        return Error::propagate(std::move(subdivided), "reproject mask");
    }

    MultipolygonWithHoles2 result;
    auto ring = rings.cbegin();
    for (const auto& polygon : polygons->polygons.polygons_with_holes()) {
        auto outer = to_polygon(*ring++, true);
        if (!outer) {
            return Error::propagate(std::move(outer), "reproject mask");
        }
        PolygonWithHoles2 reprojected(std::move(*outer));
        for (std::size_t i = 0; i < polygon.number_of_holes(); ++i) {
            auto hole = to_polygon(*ring++, false);
            if (!hole) {
                return Error::propagate(std::move(hole), "reproject mask");
            }
            reprojected.add_hole(std::move(*hole));
        }
        result.add_polygon_with_holes(std::move(reprojected));
    }

    LOG_DEBUG("Reprojected mask from {} to {} vertices", vertex_count(polygons->polygons), vertex_count(result));
    return ReferencedPolygonMask { .polygons = std::move(result), .srs = std::move(normalized_target_srs) };
}

Expected<ReferencedPolygonMask> simplify(const Expected<ReferencedPolygonMask>& polygons, const double tolerance_m)
{
    ASSERT(tolerance_m > 0);
    if (!polygons) {
        return std::unexpected(polygons.error());
    }
    auto tolerances = ToleranceGrid::make(polygons->srs, polygons->polygons.bbox(), tolerance_m);
    if (!tolerances) {
        return Error::propagate(std::move(tolerances), "simplify mask");
    }

    // All rings are constraints of one triangulation, which keeps them from crossing each other.
    SimplificationTriangulation triangulation;
    std::vector<std::vector<SimplificationTriangulation::Constraint_id>> ring_ids;
    for (const auto& polygon : polygons->polygons.polygons_with_holes()) {
        auto& ids = ring_ids.emplace_back();
        ids.push_back(insert_ring(&triangulation, polygon.outer_boundary()));
        for (const auto& hole : polygon.holes()) {
            ids.push_back(insert_ring(&triangulation, hole));
        }
    }
    CGAL::Polyline_simplification_2::simplify(
        triangulation, RelativeDeviationCost { &*tolerances }, CGAL::Polyline_simplification_2::Stop_above_cost_threshold(1));

    MultipolygonWithHoles2 result;
    for (const auto& ids : ring_ids) {
        PolygonWithHoles2 polygon(extract_ring(triangulation, ids.front()));
        for (auto id = std::next(ids.begin()); id != ids.end(); ++id) {
            polygon.add_hole(extract_ring(triangulation, *id));
        }
        result.add_polygon_with_holes(std::move(polygon));
    }

    LOG_DEBUG("Simplified mask with {} m tolerance from {} to {} vertices", tolerance_m, vertex_count(polygons->polygons), vertex_count(result));
    return ReferencedPolygonMask { .polygons = std::move(result), .srs = polygons->srs };
}

} // namespace gis::vector_mask
