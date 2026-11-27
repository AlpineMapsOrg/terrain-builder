/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2022 Adam Celarek-Litofcenko
 * Copyright (C) 2022 Martin Braunsperger
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

#include "gis/srs/coverage.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <libassert/assert.hpp>
#include <ogr_spatialref.h>

#include "error/Context.h"
#include "error/try.h"
#include "error/utils.h"
#include "geometry/bounds.h"
#include "gis/CplError.h"
#include "gis/srs/Axis.h"
#include "gis/srs/Kind.h"
#include "gis/srs/Unit.h"
#include "gis/srs/classify.h"
#include "gis/srs/detail.h"
#include "gis/srs/Transform.h"
#include "gis/srs/well_known.h"
#include "gis/webmercator.h"
#include "mesh/bounds.h"
#include "range_utils.h"

namespace gis::srs::coverage {

std::string_view format_as(const UnsupportedBounds error) {
    switch (error) {
    case UnsupportedBounds::OutsideWebMercatorExtent:
        return "the Web Mercator bounds reach beyond the world extent";
    case UnsupportedBounds::HeightOutOfRange:
        return "the heights lie outside -20 km to 200 km";
    case UnsupportedBounds::NearEarthCentre:
        return "the ECEF bounds come within 43 km of the earth's centre";
    }
    UNREACHABLE("invalid error", error);
}

namespace {
    // Points GDAL inserts between the corners of each edge when transforming bounds.
    constexpr int boundary_samples = 257;

    // Transforms bounds with GDAL, which samples the densified boundary. Extrema of curved
    // edges can lie between samples, so each side is widened by a full sampling interval.
    // Geographic bounds crossing the antimeridian are unwrapped, i.e., max.x exceeds 180.
    alp::Result<radix::tile::SrsBounds, UntransformablePoint> guarded_bounds_transform(OGRCoordinateTransformation *transform, const radix::tile::SrsBounds &bounds) {
        radix::tile::SrsBounds result;
        auto [success, gdal_error] = capture_error([&] {
            return detail::ogr_transform_bounds(transform,
                bounds.min.x,
                bounds.min.y,
                bounds.max.x,
                bounds.max.y,
                &result.min.x,
                &result.min.y,
                &result.max.x,
                &result.max.y,
                boundary_samples);
        });
        if (!success) {
            if (gdal_error) {
                return alp::make_error(UntransformablePoint{}, "transforming the bounds", std::move(*gdal_error));
            }
            return alp::make_error(UntransformablePoint{}, "transforming the bounds");
        }
        if (!geometry::is_finite(result)) {
            return alp::make_error(UntransformablePoint{}, "the transformed bounds are not finite");
        }
        // TransformBounds reports antimeridian-crossing bounds with max.x < min.x.
        if (result.max.x < result.min.x) {
            result.max.x += 360;
        }
        result = geometry::pad_bounds_relative(result, 1.0 / (boundary_samples + 1), 1e-6);
        return result;
    }

    // Splits longitude/latitude bounds into at most two bounds with longitudes in [-180, 180].
    // Bounds crossing the antimeridian may be given with max.x < min.x or max.x beyond 180.
    std::vector<radix::tile::SrsBounds> split_at_antimeridian(const radix::tile::SrsBounds& bounds) {
        const glm::bvec2 longitude_only(true, false);
        const glm::dvec2 centre(0, 0);
        const glm::dvec2 period(convert(2 * std::numbers::pi, units::radian, units::degree), 0);
        const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(bounds, longitude_only, centre, period);
        return to_vector<radix::tile::SrsBounds>(wrapped);
    }

    bool is_finite_and_ordered(const radix::geometry::Aabb3d& bounds) {
        return geometry::is_finite(bounds) && glm::all(glm::lessThanEqual(bounds.min, bounds.max));
    }

    // Distance in metres by which horizontal datum shifts at the given ellipsoidal heights in metres
    // may differ from those at height zero, at which bounds are transformed. Empirical: a survey of
    // the PROJ 9.6 operations of 27 datums to and from WGS84 found at most 0.19 mm per metre of
    // height (Pulkovo 1942); this allows 1.6 times that.
    double datum_margin(double min_height, double max_height) {
        return 0.0003 * std::max(std::abs(min_height), std::abs(max_height));
    }

    // Longitude ranges in degrees of the points in an ECEF xy rectangle. Longitude is the angle
    // of the point around the polar axis, so its range is spanned by the corners, unless the
    // rectangle contains the polar axis (all longitudes) or crosses the antimeridian (split).
    std::vector<glm::dvec2> longitude_ranges(const glm::dvec2& min, const glm::dvec2& max) {
        const auto corner_range = [](double x0, double x1, double y0, double y1) {
            glm::dvec2 range(std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest());
            for (const double x : { x0, x1 }) {
                for (const double y : { y0, y1 }) {
                    const double longitude = glm::degrees(std::atan2(y, x));
                    range = { std::min(range.x, longitude), std::max(range.y, longitude) };
                }
            }
            return range;
        };
        if (min.x <= 0 && max.x >= 0 && min.y <= 0 && max.y >= 0) {
            return { { -180, 180 } };
        }
        if (max.x < 0 && min.y < 0 && max.y >= 0) {
            // The antimeridian is the negative x half-axis; atan2 maps y = +0 to 180 and y = -0 to -180.
            return { corner_range(min.x, max.x, 0.0, max.y), corner_range(min.x, max.x, min.y, -0.0) };
        }
        return { corner_range(min.x, max.x, min.y, max.y) };
    }
} // namespace

alp::Result<std::vector<radix::tile::SrsBounds>, Error> compute_geographic(const System &reference, const radix::tile::SrsBounds &bounds) {
    ASSERT(geometry::is_finite(bounds));
    const auto context = alp::make_context("computing WGS84 bounds covering bounds in {}", reference);
    if (!is_geodetic(reference.kind())) {
        return alp::make_error(NotGeodetic{}, context);
    }
    if ((bounds.min.x < -webmercator::half_extent || bounds.max.x > webmercator::half_extent) && webmercator().is_same(reference)) {
        return alp::make_error(UnsupportedBounds::OutsideWebMercatorExtent, context);
    }
    const auto geographic = wgs84();
    radix::tile::SrsBounds longitude_latitude = bounds;
    if (!reference.is_same(geographic)) {
        ALP_TRY_CONTEXT(const Transform transform, get_transform(reference, geographic), context);
        ALP_TRY_CONTEXT(longitude_latitude, guarded_bounds_transform(transform.as_gdal(), bounds), context);
        longitude_latitude.min.y = std::max(-90.0, longitude_latitude.min.y);
        longitude_latitude.max.y = std::min(90.0, longitude_latitude.max.y);
    }
    return split_at_antimeridian(longitude_latitude);
}

alp::Result<std::vector<radix::tile::SrsBounds>, Error> compute_mercator(const System &reference, const radix::tile::SrsBounds &bounds) {
    ASSERT(geometry::is_finite(bounds));
    const auto context = alp::make_context("computing Web Mercator bounds covering bounds in {}", reference);
    const auto mercator = webmercator();
    const glm::dvec2 half_extent(webmercator::half_extent);
    // Avoid the geographic round trip, it would widen exact Web Mercator bounds by a guard band.
    if (reference.is_same(mercator) && bounds.min.x >= -webmercator::half_extent && bounds.max.x <= webmercator::half_extent) {
        const radix::tile::SrsBounds world { -half_extent, half_extent };
        const std::optional<radix::tile::SrsBounds> clipped_opt = radix::geometry::intersection_strict(bounds, world);
        if (!clipped_opt.has_value()) {
            return std::vector<radix::tile::SrsBounds> {};
        }
        return std::vector<radix::tile::SrsBounds> { *clipped_opt };
    }
    ALP_TRY_CONTEXT(const std::vector<radix::tile::SrsBounds> geographic, compute_geographic(reference, bounds), context);
    ALP_TRY_CONTEXT(const Transform transform, get_transform(wgs84(), mercator), context);
    std::vector<radix::tile::SrsBounds> result;
    for (const auto& longitude_latitude : geographic) {
        // Web Mercator diverges towards the poles; PROJ returns large finite values there.
        const double south = std::max(-webmercator::latitude_limit, longitude_latitude.min.y);
        const double north = std::min(webmercator::latitude_limit, longitude_latitude.max.y);
        if (south >= north) {
            continue;
        }
        std::array corners{glm::dvec2(longitude_latitude.min.x, south), glm::dvec2(longitude_latitude.max.x, north)};
        ALP_TRY_CONTEXT(transform.apply_inplace(&corners), context);
        const radix::tile::SrsBounds mercator_bounds{glm::clamp(corners[0], -half_extent, half_extent), glm::clamp(corners[1], -half_extent, half_extent)};
        result.push_back(mercator_bounds);
    }
    return result;
}

alp::Result<radix::geometry::Aabb3d, Error> compute_ecef(const System &reference, const radix::geometry::Aabb3d &bounds) {
    ASSERT(is_finite_and_ordered(bounds));
    const auto geocentric = ecef();
    if (reference.is_same(geocentric)) {
        return bounds;
    }
    const auto context = alp::make_context("computing ECEF bounds covering bounds in {}", reference);
    if (reference.is_compound()) {
        return alp::make_error(CompoundSystem{}, context);
    }
    if (bounds.min.z < -20'000 || bounds.max.z > 200'000) {
        return alp::make_error(UnsupportedBounds::HeightOutOfRange, context);
    }
    const radix::tile::SrsBounds horizontal_bounds(bounds);
    ALP_TRY_CONTEXT(const std::vector<radix::tile::SrsBounds> geographic, compute_geographic(reference, horizontal_bounds), context);
    // x = (N + h) cos(latitude) cos(longitude), y = (N + h) cos(latitude) sin(longitude) and
    // z = (N (1 - e^2) + h) sin(latitude). x turns at longitudes 0 and 180, y at -90 and 90,
    // both at the equator, z nowhere. Splitting a box at the turning values inside it gives
    // boxes whose extrema are at their corners, i.e., at combinations of these values.
    std::vector<glm::dvec3> candidates;
    for (const auto& box : geographic) {
        std::vector<double> longitudes { box.min.x, box.max.x };
        for (const double turn : { -90.0, 0.0, 90.0 }) {
            if (box.min.x < turn && turn < box.max.x) {
                longitudes.push_back(turn);
            }
        }
        std::vector<double> latitudes { box.min.y, box.max.y };
        if (box.min.y < 0 && 0 < box.max.y) {
            latitudes.push_back(0);
        }
        for (const double longitude : longitudes) {
            for (const double latitude : latitudes) {
                for (const double height : { bounds.min.z, bounds.max.z }) {
                    candidates.emplace_back(longitude, latitude, height);
                }
            }
        }
    }
    ALP_TRY_CONTEXT(const std::vector<glm::dvec3> points, transform_points(wgs84(), geocentric, std::move(candidates)), context);
    radix::geometry::Aabb3d result = calculate_bounds(points);
    // Allow for rounding in the transformation, and for datum shifts at the heights of the bounds.
    double margin = 1e-3;
    if (!wgs84().is_same(reference) && !webmercator().is_same(reference)) {
        margin += datum_margin(bounds.min.z, bounds.max.z);
    }
    result = geometry::pad_bounds_absolute(result, margin);
    return result;
}

alp::Result<std::vector<radix::geometry::Aabb3d>, Error> compute_from_ecef(const radix::geometry::Aabb3d &bounds, const System &reference) {
    ASSERT(is_finite_and_ordered(bounds));
    const auto geocentric = ecef();
    if (reference.is_same(geocentric)) {
        return std::vector<radix::geometry::Aabb3d> { bounds };
    }
    const auto context = alp::make_context("computing bounds in {} covering ECEF bounds", reference);
    if (!is_geodetic(reference.kind())) {
        return alp::make_error(NotGeodetic{}, context);
    }
    if (reference.is_compound()) {
        return alp::make_error(CompoundSystem{}, context);
    }
    const bool is_geographic = reference.kind() == Kind::Geographic;
    if (is_geographic && !has_horizontal_axes_in(reference, units::degree)) {
        return alp::make_error(AnglesNotDegrees{}, context);
    }
    if (is_geographic && !is_east_north_axis_order(reference)) {
        return alp::make_error(UnsupportedAxisOrder{}, context);
    }

    // Latitude and height depend only on z and the distance p to the polar axis; the box covers
    // [p_min, p_max] x [z_min, z_max]. Latitude is monotonic in p and z, height in p, and in z
    // within each hemisphere. The extrema are at the corners, the height minimum possibly at z = 0.
    // This breaks within the evolute of the meridian ellipse around the centre, which extends
    // 42.7 km from the polar axis and 42.8 km from the equatorial plane; PROJ is erratic there.
    // Elsewhere, PROJ's inverse is approximate deep inside the earth (metres at 1000 km radius).
    const radix::geometry::Aabb2d horizontal_bounds(bounds);
    const double p_min = radix::geometry::distance(horizontal_bounds, glm::dvec2(0));
    const double p_max = glm::length(glm::max(glm::abs(horizontal_bounds.min), glm::abs(horizontal_bounds.max)));
    constexpr double evolute_extent = 43000;
    if (p_min < evolute_extent && bounds.min.z < evolute_extent && bounds.max.z > -evolute_extent) {
        return alp::make_error(UnsupportedBounds::NearEarthCentre, context);
    }
    std::vector<double> heights { bounds.min.z, bounds.max.z };
    if (bounds.min.z < 0 && 0 < bounds.max.z) {
        heights.push_back(0);
    }
    std::vector<glm::dvec3> candidates;
    for (const double p : { p_min, p_max }) {
        for (const double z : heights) {
            candidates.emplace_back(p, 0, z);
        }
    }
    ALP_TRY_CONTEXT(const std::vector<glm::dvec3> points, transform_points(geocentric, wgs84(), std::move(candidates)), context);
    const radix::geometry::Aabb3d point_bounds = calculate_bounds(points);
    glm::dvec2 latitude_range(point_bounds.min.y, point_bounds.max.y);
    glm::dvec2 height_range(point_bounds.min.z, point_bounds.max.z);
    // Allow for rounding in the transformation.
    latitude_range = glm::clamp(latitude_range + glm::dvec2(-1e-9, 1e-9), -90.0, 90.0);
    height_range += glm::dvec2(-1e-3, 1e-3);

    std::vector<radix::tile::SrsBounds> geographic;
    for (const auto& longitude_range : longitude_ranges(horizontal_bounds.min, horizontal_bounds.max)) {
        geographic.push_back(
            { { std::max(-180.0, longitude_range.x - 1e-9), latitude_range.x }, { std::min(180.0, longitude_range.y + 1e-9), latitude_range.y } });
    }

    // Web Mercator x depends only on longitude and y only on latitude, so its bounds follow exactly.
    // Other SRSes get GDAL's densified boundary transformation with a guard band.
    std::vector<radix::tile::SrsBounds> covered;
    const auto wgs84_srs = wgs84();
    const auto webmercator_srs = webmercator();
    if (reference.is_same(wgs84_srs)) {
        covered = std::move(geographic);
    } else if (reference.is_same(webmercator_srs)) {
        for (const auto& box : geographic) {
            ALP_TRY_CONTEXT(const std::vector<radix::tile::SrsBounds> mercator, compute_mercator(wgs84_srs, box), context);
            covered.insert(covered.end(), mercator.begin(), mercator.end());
        }
    } else {
        // Allow for datum shifts at the heights of the box: the margin is a distance, so the
        // geographic bounds of the expanded box include all points within it.
        const double margin = datum_margin(height_range.x, height_range.y);
        const radix::geometry::Aabb3d expanded_bounds = geometry::pad_bounds_absolute(bounds, margin);
        ALP_TRY_CONTEXT(const std::vector<radix::geometry::Aabb3d> expanded, compute_from_ecef(expanded_bounds, wgs84_srs), context);
        ALP_TRY_CONTEXT(const Transform transform, get_transform(wgs84_srs, reference), context);
        for (const auto& box : expanded) {
            const radix::tile::SrsBounds horizontal_box(box);
            ALP_TRY_CONTEXT(radix::tile::SrsBounds transformed, guarded_bounds_transform(transform.as_gdal(), horizontal_box), context);
            if (!is_geographic) {
                covered.push_back(transformed);
                continue;
            }
            transformed.min.y = std::max(-90.0, transformed.min.y);
            transformed.max.y = std::min(90.0, transformed.max.y);
            const auto split = split_at_antimeridian(transformed);
            covered.insert(covered.end(), split.begin(), split.end());
        }
    }

    std::vector<radix::geometry::Aabb3d> result;
    for (const auto& box : covered) {
        result.push_back({ { box.min, height_range.x }, { box.max, height_range.y } });
    }
    return result;
}

}
