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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <fmt/core.h>
#include <gdal_priv.h>
#include <glm/glm.hpp>
#include <ogrsf_frmts.h>

#include "../result_helpers.h"
#include "enumerate.h"
#include "geometry/geometry.h"
#include "gis/Dataset.h"
#include "gis/gdal.h"
#include "gis/srs/System.h"
#include "gis/srs/Transform.h"
#include "gis/srs/well_known.h"
#include "gis/vector_mask.h"
#include "mesh/convert.h"

namespace {
using gis::vector_mask::InvalidMask;
using gis::vector_mask::Polygon2;
using gis::vector_mask::PolygonWithHoles2;
using gis::vector_mask::ReferencedPolygonMask;

struct Layer {
    std::optional<uint32_t> epsg;
    // An empty string adds a feature without geometry.
    std::vector<std::string> wkt_geometries;
};

gis::Dataset memory_dataset(const std::vector<Layer> &layers) {
    gis::gdal::initialize_once();
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("Memory");
    REQUIRE(driver);
    gis::Dataset dataset(driver->Create("", 0, 0, 0, GDT_Unknown, nullptr));
    for (const auto [i, layer_data] : enumerate(layers)) {
        std::optional<gis::srs::System> srs_opt;
        if (layer_data.epsg) {
            srs_opt = REQUIRE_VALUE(gis::srs::from_epsg(*layer_data.epsg));
        }
        OGRLayer *layer = dataset.as_gdal()->CreateLayer(fmt::format("mask{}", i).c_str(), srs_opt ? &srs_opt->as_gdal() : nullptr, wkbUnknown, nullptr);
        REQUIRE(layer);
        for (const std::string &wkt : layer_data.wkt_geometries) {
            OGRFeature feature(layer->GetLayerDefn());
            if (!wkt.empty()) {
                OGRGeometry *geometry = nullptr;
                REQUIRE(OGRGeometryFactory::createFromWkt(wkt.c_str(), nullptr, &geometry) == OGRERR_NONE);
                REQUIRE(feature.SetGeometryDirectly(geometry) == OGRERR_NONE);
            }
            REQUIRE(layer->CreateFeature(&feature) == OGRERR_NONE);
        }
    }
    return dataset;
}

// A layer of one 10 by 10 square, whose system is `srs`, and which fails to read any further feature if `fails_after_first` is set.
class ScriptedLayer final : public OGRLayer {
public:
    ScriptedLayer(const OGRSpatialReference &srs, const bool fails_after_first) : m_reference(srs), m_fails_after_first(fails_after_first) {
        m_definition.Reference();
    }

    void ResetReading() override {
        m_read = false;
    }

    OGRFeatureDefn *GetLayerDefn() override {
        return &m_definition;
    }

    OGRSpatialReference *GetSpatialRef() override {
        return &m_reference;
    }

    int TestCapability(const char *) override {
        return FALSE;
    }

    OGRFeature *GetNextFeature() override {
        if (m_read) {
            if (m_fails_after_first) {
                CPLError(CE_Failure, CPLE_AppDefined, "injected feature-read failure");
            }
            return nullptr;
        }
        m_read = true;
        auto feature = std::make_unique<OGRFeature>(&m_definition);
        OGRLinearRing ring;
        ring.addPoint(0, 0);
        ring.addPoint(10, 0);
        ring.addPoint(10, 10);
        ring.addPoint(0, 10);
        ring.addPoint(0, 0);
        OGRPolygon polygon;
        REQUIRE(polygon.addRing(&ring) == OGRERR_NONE);
        REQUIRE(feature->SetGeometry(&polygon) == OGRERR_NONE);
        return feature.release();
    }

private:
    OGRFeatureDefn m_definition{"mask"};
    OGRSpatialReference m_reference;
    bool m_fails_after_first;
    bool m_read = false;
};

class ScriptedDataset final : public GDALDataset {
public:
    ScriptedDataset(const OGRSpatialReference &srs, const bool fails_after_first) : m_layer(srs, fails_after_first) {}

    int GetLayerCount() override {
        return 1;
    }

    OGRLayer *GetLayer(const int index) override {
        return index == 0 ? &m_layer : nullptr;
    }

private:
    ScriptedLayer m_layer;
};

Polygon2 to_polygon(const std::vector<glm::dvec2> &ring) {
    Polygon2 polygon;
    for (const glm::dvec2 &point : ring) {
        polygon.push_back(convert::to_cgal_point<gis::vector_mask::Kernel>(point));
    }
    return polygon;
}

std::vector<glm::dvec2> to_points(const Polygon2 &polygon) {
    std::vector<glm::dvec2> points;
    for (const auto &point : polygon) {
        points.push_back(convert::to_glm_point(point));
    }
    return points;
}

// The first ring is the counterclockwise outer boundary, the others are clockwise holes.
ReferencedPolygonMask make_mask(const std::vector<std::vector<glm::dvec2>> &rings, const uint32_t epsg) {
    PolygonWithHoles2 polygon(to_polygon(rings.front()));
    for (size_t i = 1; i < rings.size(); i++) {
        polygon.add_hole(to_polygon(rings[i]));
    }
    ReferencedPolygonMask mask{.polygons = {}, .srs = REQUIRE_VALUE(gis::srs::from_epsg(epsg))};
    mask.polygons.add_polygon_with_holes(std::move(polygon));
    return mask;
}

const PolygonWithHoles2 &single_polygon(const ReferencedPolygonMask &mask) {
    REQUIRE(mask.polygons.number_of_polygons_with_holes() == 1);
    return mask.polygons.polygons_with_holes().front();
}

double distance_to_ring(const glm::dvec2 &point, const std::vector<glm::dvec2> &ring) {
    double nearest = std::numeric_limits<double>::max();
    for (size_t i = 0; i < ring.size(); i++) {
        nearest = std::min(nearest, geometry::distance_to_segment(point, radix::geometry::Edge<2, double>{ring[i], ring[(i + 1) % ring.size()]}));
    }
    return nearest;
}

// A square whose bottom edge has a middle vertex offset outwards.
std::vector<glm::dvec2> square_with_offset_vertex(const glm::dvec2 &origin, const double size, const double offset) {
    return {origin, origin + glm::dvec2(size / 2, -offset), origin + glm::dvec2(size, 0), origin + glm::dvec2(size, size), origin + glm::dvec2(0, size)};
}
} // namespace

TEST_CASE("load keeps polygons and orients their rings", "[gis][vector_mask]") {
    auto dataset = memory_dataset({{3035,
        {
            "POLYGON ((0 0,0 100,100 100,100 0,0 0),(30 30,70 30,70 70,30 70,30 30))",
            "MULTIPOLYGON (((200 0,300 0,300 100,200 100,200 0)),((400 0,500 0,500 100,400 100,400 0)))",
        }}});
    const ReferencedPolygonMask loaded = REQUIRE_VALUE(gis::vector_mask::load(dataset));
    CHECK(loaded.srs.is_same(REQUIRE_VALUE(gis::srs::from_epsg(3035))));
    REQUIRE(loaded.polygons.number_of_polygons_with_holes() == 3);
    const PolygonWithHoles2 &polygon = loaded.polygons.polygons_with_holes().front();
    CHECK(polygon.outer_boundary().size() == 4);
    CHECK(polygon.outer_boundary().is_counterclockwise_oriented());
    REQUIRE(polygon.number_of_holes() == 1);
    CHECK(polygon.holes().front().size() == 4);
    CHECK(polygon.holes().front().is_clockwise_oriented());
}

TEST_CASE("load assumes WGS84 without a spatial reference system", "[gis][vector_mask]") {
    auto dataset = memory_dataset({{std::nullopt, {"POLYGON ((10 46,11 46,11 47,10 47,10 46))"}}});
    const ReferencedPolygonMask loaded = REQUIRE_VALUE(gis::vector_mask::load(dataset));
    CHECK(loaded.srs.is_same(gis::srs::wgs84()));
}

TEST_CASE("load fails as an invalid mask for broken features", "[gis][vector_mask]") {
    const std::string square = "POLYGON ((0 0,100 0,100 100,0 100,0 0))";
    std::optional<gis::Dataset> dataset_opt;
    InvalidMask expected = InvalidMask::MissingGeometry;
    SECTION("self-intersecting ring") {
        dataset_opt = memory_dataset({{3035, {"POLYGON ((0 0,100 100,100 0,0 100,0 0))"}}});
        expected = InvalidMask::InvalidGeometry;
    }
    SECTION("degenerate ring") {
        dataset_opt = memory_dataset({{3035, {"POLYGON ((0 0,100 100,100 100,0 0))"}}});
        expected = InvalidMask::InvalidGeometry;
    }
    SECTION("feature without geometry") {
        dataset_opt = memory_dataset({{3035, {square, ""}}});
        expected = InvalidMask::InvalidGeometry;
    }
    SECTION("line geometry") {
        dataset_opt = memory_dataset({{3035, {"LINESTRING (0 0,100 100)"}}});
        expected = InvalidMask::UnsupportedGeometry;
    }
    SECTION("layers with different spatial reference systems") {
        dataset_opt = memory_dataset({{3035, {square}}, {31287, {square}}});
        expected = InvalidMask::AmbiguousSystem;
    }
    SECTION("no polygons") {
        dataset_opt = memory_dataset({{3035, {}}});
        expected = InvalidMask::MissingGeometry;
    }

    const auto error = REQUIRE_ERROR(gis::vector_mask::load(*dataset_opt));

    CHECK(error.holds_code(expected));
}

TEST_CASE("load fails as unsupported axis order for a layer that orders its axes north, east", "[gis][vector_mask]") {
    OGRSpatialReference north_east;
    REQUIRE(north_east.importFromEPSG(4326) == OGRERR_NONE);
    north_east.SetAxisMappingStrategy(OAMS_AUTHORITY_COMPLIANT);
    gis::Dataset dataset(new ScriptedDataset(north_east, false));

    const auto error = REQUIRE_ERROR(gis::vector_mask::load(dataset));

    CHECK(error.holds<gis::srs::UnsupportedAxisOrder>());
}

TEST_CASE("load fails as unreadable when GDAL fails to read a feature", "[gis][vector_mask]") {
    gis::Dataset dataset(new ScriptedDataset(gis::srs::webmercator().as_gdal(), true));

    const auto error = REQUIRE_ERROR(gis::vector_mask::load(dataset));

    CHECK(error.holds<gis::vector_mask::UnreadableMask>());
}

TEST_CASE("simplify removes vertices within the tolerance", "[gis][vector_mask]") {
    // A 1 km square in the Alps with 1000 vertices per side, each up to 2 cm off the straight side.
    const glm::dvec2 origin = {4'300'000, 2'600'000};
    const std::vector<glm::dvec2> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    std::vector<glm::dvec2> ring;
    glm::dvec2 corner = origin;
    for (const glm::dvec2 &direction : directions) {
        for (int32_t i = 0; i < 1000; i++) {
            const glm::dvec2 normal = {direction.y, -direction.x};
            ring.push_back(corner + direction * static_cast<double>(i) + (i == 0 ? 0.0 : 0.02 * std::sin(i * 0.7)) * normal);
        }
        corner += direction * 1000.0;
    }

    const ReferencedPolygonMask simplified = REQUIRE_VALUE(gis::vector_mask::simplify(make_mask({ring}, 3035), 0.1));
    CHECK(simplified.srs.is_same(REQUIRE_VALUE(gis::srs::from_epsg(3035))));
    const std::vector<glm::dvec2> result = to_points(single_polygon(simplified).outer_boundary());
    CHECK(result.size() <= 8);
    double max_deviation = 0;
    for (const glm::dvec2 &point : ring) {
        max_deviation = std::max(max_deviation, distance_to_ring(point, result));
    }
    CHECK(max_deviation <= 0.1);
}

TEST_CASE("simplify keeps rings from crossing", "[gis][vector_mask]") {
    // Removing the peak of the outer boundary would cut through the hole below it.
    const glm::dvec2 origin = {4'300'000, 2'600'000};
    const std::vector<glm::dvec2> outer = {origin, origin + glm::dvec2(100, 0), origin + glm::dvec2(100, 100), origin + glm::dvec2(50, 102), origin + glm::dvec2(0, 100)};
    const std::vector<glm::dvec2> hole = {origin + glm::dvec2(45, 99), origin + glm::dvec2(45, 101), origin + glm::dvec2(55, 101), origin + glm::dvec2(55, 99)};

    SECTION("without the hole the peak goes") {
        const ReferencedPolygonMask simplified = REQUIRE_VALUE(gis::vector_mask::simplify(make_mask({outer}, 3035), 3));
        CHECK(single_polygon(simplified).outer_boundary().size() == 4);
    }
    SECTION("with the hole the peak stays") {
        const ReferencedPolygonMask simplified = REQUIRE_VALUE(gis::vector_mask::simplify(make_mask({outer, hole}, 3035), 3));
        const PolygonWithHoles2 &polygon = single_polygon(simplified);
        CHECK(polygon.outer_boundary().size() == 5);
        REQUIRE(polygon.number_of_holes() == 1);
        for (const auto &point : polygon.holes().front()) {
            CHECK(polygon.outer_boundary().bounded_side(point) == CGAL::ON_BOUNDED_SIDE);
        }
    }
}

TEST_CASE("simplify follows the local scale with its tolerance", "[gis][vector_mask]") {
    const auto vertex_count = [](const ReferencedPolygonMask &mask, const double tolerance_m) {
        const ReferencedPolygonMask simplified = REQUIRE_VALUE(gis::vector_mask::simplify(mask, tolerance_m));
        return single_polygon(simplified).outer_boundary().size();
    };

    SECTION("web mercator, where a unit is half a metre at 60 degrees latitude") {
        const glm::dvec2 equator = REQUIRE_VALUE(gis::srs::transform_point(gis::srs::wgs84(), gis::srs::webmercator(), glm::dvec2(10, 0)));
        const glm::dvec2 north = REQUIRE_VALUE(gis::srs::transform_point(gis::srs::wgs84(), gis::srs::webmercator(), glm::dvec2(10, 60)));
        CHECK(vertex_count(make_mask({square_with_offset_vertex(equator, 100, 0.05)}, 3857), 0.1) == 4);
        CHECK(vertex_count(make_mask({square_with_offset_vertex(equator, 100, 0.15)}, 3857), 0.1) == 5);
        CHECK(vertex_count(make_mask({square_with_offset_vertex(north, 100, 0.15)}, 3857), 0.1) == 4);
        CHECK(vertex_count(make_mask({square_with_offset_vertex(north, 100, 0.25)}, 3857), 0.1) == 5);
    }
    SECTION("geographic, with the offset along a meridian") {
        // About the length of a degree of latitude at 47 degrees.
        const double degrees_per_metre = 1 / 111'200.0;
        const auto ring = [&](const double offset_m) { return square_with_offset_vertex({10, 47}, 100 * degrees_per_metre, offset_m * degrees_per_metre); };
        CHECK(vertex_count(make_mask({ring(0.05)}, 4326), 0.1) == 4);
        CHECK(vertex_count(make_mask({ring(0.2)}, 4326), 0.1) == 5);
    }
}

TEST_CASE("transform subdivides long edges within the tolerance", "[gis][vector_mask]") {
    const std::vector<glm::dvec2> ring = {{10, 46}, {12, 46}, {12, 48}, {10, 48}};
    const gis::srs::System target_srs = REQUIRE_VALUE(gis::srs::from_epsg(3035));
    const double tolerance_m = 0.5;
    const ReferencedPolygonMask reprojected = REQUIRE_VALUE(gis::vector_mask::transform(make_mask({ring}, 4326), target_srs, tolerance_m));
    CHECK(reprojected.srs.is_same(target_srs));
    const Polygon2 &outer = single_polygon(reprojected).outer_boundary();
    CHECK(outer.is_counterclockwise_oriented());
    CHECK(outer.size() > 4);
    CHECK(outer.size() < 2000);

    // Densely sampled source edges, reprojected, stay within the tolerance of the result.
    std::vector<glm::dvec2> reference;
    for (size_t i = 0; i < ring.size(); i++) {
        for (int32_t step = 0; step < 1000; step++) {
            reference.push_back(glm::mix(ring[i], ring[(i + 1) % ring.size()], step / 1000.0));
        }
    }
    const std::vector<glm::dvec2> reprojected_reference = REQUIRE_VALUE(gis::srs::transform_points(gis::srs::wgs84(), target_srs, reference));
    const std::vector<glm::dvec2> result = to_points(outer);
    double max_deviation = 0;
    for (const glm::dvec2 &point : reprojected_reference) {
        max_deviation = std::max(max_deviation, distance_to_ring(point, result));
    }
    CHECK(max_deviation <= tolerance_m * 1.05);
}

TEST_CASE("transform keeps short edges", "[gis][vector_mask]") {
    const std::vector<glm::dvec2> ring = {{10, 46}, {10.001, 46}, {10.001, 46.001}, {10, 46.001}};
    const ReferencedPolygonMask reprojected = REQUIRE_VALUE(gis::vector_mask::transform(make_mask({ring}, 4326), REQUIRE_VALUE(gis::srs::from_epsg(3035)), 0.1));
    CHECK(single_polygon(reprojected).outer_boundary().size() == 4);
}

TEST_CASE("transform fails as unreprojectable for a target GDAL cannot reach", "[gis][vector_mask]") {
    const std::vector<glm::dvec2> ring = {{10, 46}, {11, 46}, {11, 47}, {10, 47}};
    const gis::srs::System mars = REQUIRE_VALUE(gis::srs::from_user_input("IAU_2015:49900"));

    const auto error = REQUIRE_ERROR(gis::vector_mask::transform(make_mask({ring}, 4326), mars, 1));

    CHECK(error.holds<gis::vector_mask::UnreprojectableMask>());
}

TEST_CASE("transform and simplify fail as unsupported tolerance srs for a geocentric mask", "[gis][vector_mask]") {
    const std::vector<glm::dvec2> ring = {{10, 46}, {11, 46}, {11, 47}, {10, 47}};
    SECTION("transform") {
        const auto error = REQUIRE_ERROR(gis::vector_mask::transform(make_mask({ring}, 4326), gis::srs::ecef(), 1));

        CHECK(error.holds<gis::vector_mask::UnsupportedToleranceSrs>());
    }
    SECTION("simplify") {
        const auto error = REQUIRE_ERROR(gis::vector_mask::simplify(make_mask({ring}, 4978), 1));

        CHECK(error.holds<gis::vector_mask::UnsupportedToleranceSrs>());
    }
}

TEST_CASE("simplify keeps the vertices transform adds along curved edges", "[gis][vector_mask]") {
    // Parallels are curved in EPSG:3035, so subdivided vertices survive the simplification.
    auto dataset = memory_dataset({{4326, {"POLYGON ((10 46,12 46,12 48,10 48,10 46))"}}});
    const gis::srs::System target_srs = REQUIRE_VALUE(gis::srs::from_epsg(3035));
    const ReferencedPolygonMask loaded = REQUIRE_VALUE(gis::vector_mask::load(dataset));
    const ReferencedPolygonMask reprojected = REQUIRE_VALUE(gis::vector_mask::transform(loaded, target_srs, 0.5));

    const ReferencedPolygonMask mask = REQUIRE_VALUE(gis::vector_mask::simplify(reprojected, 0.5));

    CHECK(mask.srs.is_same(target_srs));
    CHECK(single_polygon(mask).outer_boundary().size() > 4);
}
