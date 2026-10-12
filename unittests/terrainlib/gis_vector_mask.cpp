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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <fmt/core.h>
#include <gdal_priv.h>
#include <glm/glm.hpp>
#include <ogrsf_frmts.h>

#include "geometry/geometry.h"
#include "gis/Dataset.h"
#include "gis/gdal.h"
#include "gis/srs.h"
#include "gis/vector_mask.h"
#include "mesh/convert.h"

namespace {
using gis::vector_mask::Polygon2;
using gis::vector_mask::PolygonWithHoles2;
using gis::vector_mask::ReferencedPolygonMask;

struct Layer {
    std::optional<int> epsg;
    std::vector<std::string> wkt_geometries;
};

gis::Dataset memory_dataset(const std::vector<Layer>& layers)
{
    gis::gdal::initialize_once();
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("Memory");
    REQUIRE(driver);
    gis::Dataset dataset(driver->Create("", 0, 0, 0, GDT_Unknown, nullptr));
    for (std::size_t i = 0; i < layers.size(); ++i) {
        OGRSpatialReference srs;
        if (layers[i].epsg) {
            REQUIRE(srs.importFromEPSG(*layers[i].epsg) == OGRERR_NONE);
        }
        OGRLayer* layer = dataset.gdalDataset()->CreateLayer(fmt::format("mask{}", i).c_str(), layers[i].epsg ? &srs : nullptr, wkbUnknown, nullptr);
        REQUIRE(layer);
        for (const std::string& wkt : layers[i].wkt_geometries) {
            OGRGeometry* geometry = nullptr;
            REQUIRE(OGRGeometryFactory::createFromWkt(wkt.c_str(), nullptr, &geometry) == OGRERR_NONE);
            OGRFeature feature(layer->GetLayerDefn());
            REQUIRE(feature.SetGeometryDirectly(geometry) == OGRERR_NONE);
            REQUIRE(layer->CreateFeature(&feature) == OGRERR_NONE);
        }
    }
    return dataset;
}

Polygon2 to_polygon(const std::vector<glm::dvec2>& ring)
{
    Polygon2 polygon;
    for (const glm::dvec2& point : ring) {
        polygon.push_back(convert::to_cgal_point<gis::vector_mask::Kernel>(point));
    }
    return polygon;
}

std::vector<glm::dvec2> to_points(const Polygon2& polygon)
{
    std::vector<glm::dvec2> points;
    for (const auto& point : polygon) {
        points.push_back(convert::to_glm_point(point));
    }
    return points;
}

// The first ring is the counterclockwise outer boundary, the others are clockwise holes.
ReferencedPolygonMask make_mask(const std::vector<std::vector<glm::dvec2>>& rings, const int epsg)
{
    PolygonWithHoles2 polygon(to_polygon(rings.front()));
    for (std::size_t i = 1; i < rings.size(); ++i) {
        polygon.add_hole(to_polygon(rings[i]));
    }
    ReferencedPolygonMask mask { .polygons = {}, .srs = gis::srs::from_epsg(epsg).value() };
    mask.polygons.add_polygon_with_holes(std::move(polygon));
    return mask;
}

const PolygonWithHoles2& single_polygon(const ReferencedPolygonMask& mask)
{
    REQUIRE(mask.polygons.number_of_polygons_with_holes() == 1);
    return mask.polygons.polygons_with_holes().front();
}

double distance_to_ring(const glm::dvec2& point, const std::vector<glm::dvec2>& ring)
{
    double nearest = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < ring.size(); ++i) {
        nearest = std::min(nearest, geometry::distance_to_segment(point, radix::geometry::Edge<2, double> { ring[i], ring[(i + 1) % ring.size()] }));
    }
    return nearest;
}

// A square whose bottom edge has a middle vertex offset outwards.
std::vector<glm::dvec2> square_with_offset_vertex(const glm::dvec2& origin, const double size, const double offset)
{
    return { origin, origin + glm::dvec2(size / 2, -offset), origin + glm::dvec2(size, 0), origin + glm::dvec2(size, size), origin + glm::dvec2(0, size) };
}

bool has_srs(const ReferencedPolygonMask& mask, const OGRSpatialReference& srs) { return mask.srs.IsSame(&srs); }
} // namespace

TEST_CASE("vector mask load keeps polygons and orients their rings", "[gis][vector_mask]")
{
    auto dataset = memory_dataset({ { 3035,
        {
            "POLYGON ((0 0,0 100,100 100,100 0,0 0),(30 30,70 30,70 70,30 70,30 30))",
            "MULTIPOLYGON (((200 0,300 0,300 100,200 100,200 0)),((400 0,500 0,500 100,400 100,400 0)))",
        } } });
    const auto loaded = gis::vector_mask::load(dataset);
    REQUIRE(loaded);
    CHECK(has_srs(*loaded, gis::srs::from_epsg(3035).value()));
    REQUIRE(loaded->polygons.number_of_polygons_with_holes() == 3);
    const PolygonWithHoles2& polygon = loaded->polygons.polygons_with_holes().front();
    CHECK(polygon.outer_boundary().size() == 4);
    CHECK(polygon.outer_boundary().is_counterclockwise_oriented());
    REQUIRE(polygon.number_of_holes() == 1);
    CHECK(polygon.holes().front().size() == 4);
    CHECK(polygon.holes().front().is_clockwise_oriented());
}

TEST_CASE("vector mask load assumes WGS84 without an SRS", "[gis][vector_mask]")
{
    auto dataset = memory_dataset({ { std::nullopt, { "POLYGON ((10 46,11 46,11 47,10 47,10 46))" } } });
    const auto loaded = gis::vector_mask::load(dataset);
    REQUIRE(loaded);
    CHECK(has_srs(*loaded, gis::srs::wgs84()));
}

TEST_CASE("vector mask load rejects broken input", "[gis][vector_mask]")
{
    const std::string square = "POLYGON ((0 0,100 0,100 100,0 100,0 0))";
    SECTION("self-intersecting ring")
    {
        auto dataset = memory_dataset({ { 3035, { "POLYGON ((0 0,100 100,100 0,0 100,0 0))" } } });
        const auto loaded = gis::vector_mask::load(dataset);
        REQUIRE_FALSE(loaded);
        CHECK(loaded.error().code() == Error::Code::CorruptData);
    }
    SECTION("degenerate ring")
    {
        auto dataset = memory_dataset({ { 3035, { "POLYGON ((0 0,100 100,100 100,0 0))" } } });
        const auto loaded = gis::vector_mask::load(dataset);
        REQUIRE_FALSE(loaded);
        CHECK(loaded.error().code() == Error::Code::CorruptData);
    }
    SECTION("layers with different SRSs")
    {
        auto dataset = memory_dataset({ { 3035, { square } }, { 31287, { square } } });
        const auto loaded = gis::vector_mask::load(dataset);
        REQUIRE_FALSE(loaded);
        CHECK(loaded.error().code() == Error::Code::Unsupported);
    }
    SECTION("line geometry")
    {
        auto dataset = memory_dataset({ { 3035, { "LINESTRING (0 0,100 100)" } } });
        const auto loaded = gis::vector_mask::load(dataset);
        REQUIRE_FALSE(loaded);
        CHECK(loaded.error().code() == Error::Code::Unsupported);
    }
    SECTION("no polygons")
    {
        auto dataset = memory_dataset({ { 3035, {} } });
        const auto loaded = gis::vector_mask::load(dataset);
        REQUIRE_FALSE(loaded);
        CHECK(loaded.error().code() == Error::Code::InvalidInput);
    }
}

TEST_CASE("vector mask simplification removes vertices within the tolerance", "[gis][vector_mask]")
{
    // A 1 km square in the Alps with 1000 vertices per side, each up to 2 cm off the straight side.
    const glm::dvec2 origin = { 4'300'000, 2'600'000 };
    const std::vector<glm::dvec2> directions = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } };
    std::vector<glm::dvec2> ring;
    glm::dvec2 corner = origin;
    for (const glm::dvec2& direction : directions) {
        for (int i = 0; i < 1000; ++i) {
            const glm::dvec2 normal = { direction.y, -direction.x };
            ring.push_back(corner + direction * double(i) + (i == 0 ? 0.0 : 0.02 * std::sin(i * 0.7)) * normal);
        }
        corner += direction * 1000.0;
    }

    const auto simplified = gis::vector_mask::simplify(make_mask({ ring }, 3035), 0.1);
    REQUIRE(simplified);
    CHECK(has_srs(*simplified, gis::srs::from_epsg(3035).value()));
    const std::vector<glm::dvec2> result = to_points(single_polygon(*simplified).outer_boundary());
    CHECK(result.size() <= 8);
    double max_deviation = 0;
    for (const glm::dvec2& point : ring) {
        max_deviation = std::max(max_deviation, distance_to_ring(point, result));
    }
    CHECK(max_deviation <= 0.1);
}

TEST_CASE("vector mask simplification keeps rings from crossing", "[gis][vector_mask]")
{
    // Removing the peak of the outer boundary would cut through the hole below it.
    const glm::dvec2 origin = { 4'300'000, 2'600'000 };
    const std::vector<glm::dvec2> outer
        = { origin, origin + glm::dvec2(100, 0), origin + glm::dvec2(100, 100), origin + glm::dvec2(50, 102), origin + glm::dvec2(0, 100) };
    const std::vector<glm::dvec2> hole
        = { origin + glm::dvec2(45, 99), origin + glm::dvec2(45, 101), origin + glm::dvec2(55, 101), origin + glm::dvec2(55, 99) };

    SECTION("without the hole the peak goes")
    {
        const auto simplified = gis::vector_mask::simplify(make_mask({ outer }, 3035), 3);
        REQUIRE(simplified);
        CHECK(single_polygon(*simplified).outer_boundary().size() == 4);
    }
    SECTION("with the hole the peak stays")
    {
        const auto simplified = gis::vector_mask::simplify(make_mask({ outer, hole }, 3035), 3);
        REQUIRE(simplified);
        const PolygonWithHoles2& polygon = single_polygon(*simplified);
        CHECK(polygon.outer_boundary().size() == 5);
        REQUIRE(polygon.number_of_holes() == 1);
        for (const auto& point : polygon.holes().front()) {
            CHECK(polygon.outer_boundary().bounded_side(point) == CGAL::ON_BOUNDED_SIDE);
        }
    }
}

TEST_CASE("vector mask simplification tolerance follows the local scale", "[gis][vector_mask]")
{
    const auto vertex_count = [](const ReferencedPolygonMask& mask, const double tolerance_m) {
        const auto simplified = gis::vector_mask::simplify(mask, tolerance_m);
        REQUIRE(simplified);
        return single_polygon(*simplified).outer_boundary().size();
    };

    SECTION("web mercator, where a unit is half a metre at 60 degrees latitude")
    {
        const auto equator = gis::srs::transform_point(gis::srs::wgs84(), gis::srs::webmercator(), glm::dvec2(10, 0));
        const auto north = gis::srs::transform_point(gis::srs::wgs84(), gis::srs::webmercator(), glm::dvec2(10, 60));
        REQUIRE(equator);
        REQUIRE(north);
        CHECK(vertex_count(make_mask({ square_with_offset_vertex(*equator, 100, 0.05) }, 3857), 0.1) == 4);
        CHECK(vertex_count(make_mask({ square_with_offset_vertex(*equator, 100, 0.15) }, 3857), 0.1) == 5);
        CHECK(vertex_count(make_mask({ square_with_offset_vertex(*north, 100, 0.15) }, 3857), 0.1) == 4);
        CHECK(vertex_count(make_mask({ square_with_offset_vertex(*north, 100, 0.25) }, 3857), 0.1) == 5);
    }
    SECTION("geographic, with the offset along a meridian")
    {
        // About the length of a degree of latitude at 47 degrees.
        const double degrees_per_metre = 1 / 111'200.0;
        const auto ring = [&](const double offset_m) { return square_with_offset_vertex({ 10, 47 }, 100 * degrees_per_metre, offset_m * degrees_per_metre); };
        CHECK(vertex_count(make_mask({ ring(0.05) }, 4326), 0.1) == 4);
        CHECK(vertex_count(make_mask({ ring(0.2) }, 4326), 0.1) == 5);
    }
}

TEST_CASE("vector mask reprojection subdivides long edges within the tolerance", "[gis][vector_mask]")
{
    const std::vector<glm::dvec2> ring = { { 10, 46 }, { 12, 46 }, { 12, 48 }, { 10, 48 } };
    const OGRSpatialReference target_srs = gis::srs::from_epsg(3035).value();
    const double tolerance_m = 0.5;
    const auto reprojected = gis::vector_mask::transform(make_mask({ ring }, 4326), target_srs, tolerance_m);
    REQUIRE(reprojected);
    CHECK(has_srs(*reprojected, target_srs));
    const Polygon2& outer = single_polygon(*reprojected).outer_boundary();
    CHECK(outer.is_counterclockwise_oriented());
    CHECK(outer.size() > 4);
    CHECK(outer.size() < 2000);

    // Densely sampled source edges, reprojected, stay within the tolerance of the result.
    std::vector<glm::dvec2> reference;
    for (std::size_t i = 0; i < ring.size(); ++i) {
        for (int step = 0; step < 1000; ++step) {
            reference.push_back(glm::mix(ring[i], ring[(i + 1) % ring.size()], step / 1000.0));
        }
    }
    const auto reprojected_reference = gis::srs::transform_points(gis::srs::wgs84(), target_srs, reference);
    REQUIRE(reprojected_reference);
    const std::vector<glm::dvec2> result = to_points(outer);
    double max_deviation = 0;
    for (const glm::dvec2& point : *reprojected_reference) {
        max_deviation = std::max(max_deviation, distance_to_ring(point, result));
    }
    CHECK(max_deviation <= tolerance_m * 1.05);
}

TEST_CASE("vector mask reprojection keeps short edges", "[gis][vector_mask]")
{
    const std::vector<glm::dvec2> ring = { { 10, 46 }, { 10.001, 46 }, { 10.001, 46.001 }, { 10, 46.001 } };
    const auto reprojected = gis::vector_mask::transform(make_mask({ ring }, 4326), gis::srs::from_epsg(3035).value(), 0.1);
    REQUIRE(reprojected);
    CHECK(single_polygon(*reprojected).outer_boundary().size() == 4);
}

TEST_CASE("vector mask functions pass errors through", "[gis][vector_mask]")
{
    const Expected<ReferencedPolygonMask> failed = std::unexpected(Error::make(Error::Code::NotFound, "find mask"));
    const auto reprojected = gis::vector_mask::transform(failed, gis::srs::webmercator(), 1);
    REQUIRE_FALSE(reprojected);
    CHECK(reprojected.error().code() == Error::Code::NotFound);
    const auto simplified = gis::vector_mask::simplify(failed, 1);
    REQUIRE_FALSE(simplified);
    CHECK(simplified.error().code() == Error::Code::NotFound);
}

TEST_CASE("vector mask functions chain from load", "[gis][vector_mask]")
{
    // Parallels are curved in EPSG:3035, so subdivided vertices survive the simplification.
    auto dataset = memory_dataset({ { 4326, { "POLYGON ((10 46,12 46,12 48,10 48,10 46))" } } });
    const OGRSpatialReference target_srs = gis::srs::from_epsg(3035).value();
    const auto mask = gis::vector_mask::simplify(gis::vector_mask::transform(gis::vector_mask::load(dataset), target_srs, 0.5), 0.5);
    REQUIRE(mask);
    CHECK(has_srs(*mask, target_srs));
    CHECK(single_polygon(*mask).outer_boundary().size() > 4);
}
