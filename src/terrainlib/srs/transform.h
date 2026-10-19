#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <libassert/assert.hpp>
#include <ogr_spatialref.h>

#include "containers/HybridVector.h"
#include "enumerate.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "gdal/CplError.h"
#include "gdal/init.h"
#include "srs/System.h"
#include "srs/error.h"

namespace srs {

// A 2D or 3D point in double precision.
template <typename P>
concept Point = std::same_as<P, glm::dvec2> || std::same_as<P, glm::dvec3>;

// A range of points with a known size.
template <typename R>
concept PointRange = std::ranges::forward_range<R> && std::ranges::sized_range<R> && Point<std::ranges::range_value_t<R>>;

// A range of points that can be overwritten.
template <typename R>
concept WritablePointRange = PointRange<R> && std::ranges::output_range<R, std::ranges::range_value_t<R>>;

namespace detail {

// One coordinate of every point of a range, which stays off the heap for a few points.
using CoordinateBuffer = HybridVector<double, 8>;
// Whether GDAL mapped each point of a range, which stays off the heap for a few points.
using SuccessBuffer = HybridVector<int, 8>;

inline MissingTransform make_missing_transform(const System &source_srs, const System &target_srs, std::optional<gdal::CplError> gdal_error) {
    return MissingTransform{
        .source = friendly_name(source_srs),
        .target = friendly_name(target_srs),
        .gdal_error = std::move(gdal_error)
    };
}

// Creates the code for a point that `transform` cannot map.
inline Untransformable make_untransformable(const OGRCoordinateTransformation *transform, std::optional<gdal::CplError> gdal_error) {
    const System *source_srs = ASSERT_VAL(transform->GetSourceCS());
    const System *target_srs = ASSERT_VAL(transform->GetTargetCS());
    return Untransformable{
        .source = friendly_name(*source_srs),
        .target = friendly_name(*target_srs),
        .gdal_error = std::move(gdal_error)
    };
}
} // namespace detail

// A transform from one spatial reference to another, which must only be used on the thread it was requested on.
class Transform {
public:
    explicit Transform(std::shared_ptr<OGRCoordinateTransformation> gdal_transform) : _transform(std::move(gdal_transform)) {}

    // The GDAL transformation this wraps.
    [[nodiscard]]
    OGRCoordinateTransformation *as_gdal() const {
        return this->_transform.get();
    }

    // Maps `point` into the target reference.
    // Fails if it has no image there.
    template <Point P>
    [[nodiscard]]
    alp::Result<P, Untransformable> apply(P point) const {
        std::span<P, 1> points(&point, 1);
        ALP_TRY(this->apply_inplace(points));
        return point;
    }

    // Maps every point of `points` into the target reference.
    // Fails if one of them has no image there.
    template <WritablePointRange Points>
    [[nodiscard]]
    alp::Result<void, Untransformable> apply_inplace(Points &points) const {
        return this->apply_into(points, points);
    }

    // Maps every point of `points` into the target reference and writes the results to `out`, which must hold as many points.
    // Fails if one of them has no image there.
    template <PointRange Points, WritablePointRange Out>
        requires(std::ranges::range_value_t<Out>::length() <= std::ranges::range_value_t<Points>::length())
    [[nodiscard]]
    alp::Result<void, Untransformable> apply_into(const Points &points, Out &out) const {
        using OutPoint = std::ranges::range_value_t<Out>;
        constexpr bool is_source_3d = (std::ranges::range_value_t<Points>::length() == 3);
        constexpr bool is_target_3d = (OutPoint::length() == 3);

        const size_t point_count = std::ranges::size(points);
        ASSERT(std::ranges::size(out) == point_count);
        if (point_count == 0) {
            // Early return since GDAL reports an empty range as a failure.
            return {};
        }

        // GDAL takes every coordinate as its own array.
        detail::CoordinateBuffer xs;
        detail::CoordinateBuffer ys;
        detail::CoordinateBuffer zs;
        xs.resize(point_count);
        ys.resize(point_count);
        if constexpr (is_source_3d) {
            zs.resize(point_count);
        }
        for (const auto [i, point] : enumerate(points)) {
            xs[i] = point.x;
            ys[i] = point.y;
            if constexpr (is_source_3d) {
                zs[i] = point.z;
            }
        }

        detail::SuccessBuffer successes;
        successes.resize(point_count);
        auto [success, gdal_error] = gdal::capture_error([&] {
            if constexpr (is_source_3d) {
                return this->_transform->Transform(point_count, xs.data(), ys.data(), zs.data(), successes.data());
            } else {
                return this->_transform->Transform(point_count, xs.data(), ys.data(), nullptr, successes.data());
            }
        });
        if (!success || !std::ranges::all_of(successes, std::identity())) {
            return alp::make_unexpected(detail::make_untransformable(this->_transform.get(), std::move(gdal_error)));
        }

        for (auto [i, out_point] : enumerate(out)) {
            if constexpr (is_target_3d) {
                out_point = OutPoint(xs[i], ys[i], zs[i]);
            } else {
                out_point = OutPoint(xs[i], ys[i]);
            }
        }
        return {};
    }

private:
    std::shared_ptr<OGRCoordinateTransformation> _transform;
};

namespace detail {

struct TransformCacheEntry {
    System source;
    System target;
    Transform transform;
};

} // namespace detail

// Creates a transform from `source_srs` to `target_srs` that no other request receives.
// Fails if GDAL has no transform between them.
[[nodiscard]]
inline alp::Result<Transform, MissingTransform> create_transform(const System &source_srs, const System &target_srs) {
    gdal::initialize_once();
    auto [created, gdal_error] = gdal::capture_error([&] {
        return OGRCreateCoordinateTransformation(&source_srs, &target_srs);
    });
    std::shared_ptr<OGRCoordinateTransformation> gdal_transform(created);
    if (!gdal_transform) {
        return alp::make_unexpected(detail::make_missing_transform(source_srs, target_srs, std::move(gdal_error)));
    }
    return Transform(std::move(gdal_transform));
}

// Gives the calling thread its transform from `source_srs` to `target_srs`.
// Fails if GDAL has no transform between them.
[[nodiscard]]
inline alp::Result<Transform, MissingTransform> get_transform(const System &source_srs, const System &target_srs) {
    thread_local std::vector<detail::TransformCacheEntry> cache;

    // Check forward
    for (auto &entry : cache) {
        if (entry.source.IsSame(&source_srs) && entry.target.IsSame(&target_srs)) {
            return entry.transform;
        }
    }

    // Check inverse
    for (auto &entry : cache) {
        if (entry.source.IsSame(&target_srs) && entry.target.IsSame(&source_srs)) {
            auto [inverted, gdal_error] = gdal::capture_error([&] {
                return entry.transform.as_gdal()->GetInverse();
            });
            std::shared_ptr<OGRCoordinateTransformation> inverse(inverted);
            if (!inverse) {
                return alp::make_unexpected(detail::make_missing_transform(source_srs, target_srs, std::move(gdal_error)));
            }
            const Transform transform(std::move(inverse));
            cache.push_back({source_srs, target_srs, transform});
            return transform;
        }
    }

    // Not found
    ALP_TRY(const Transform transform, create_transform(source_srs, target_srs));
    cache.push_back({source_srs, target_srs, transform});
    return transform;
}

// TODO: a region at the antimeridian gets wrong bounds and wrong UVs from these transforms.
// Maps `point` from `source_srs` into `target_srs`.
// Fails if GDAL has no transform between them or the point has no image there.
template <Point P>
[[nodiscard]]
inline alp::Result<P, TransformError> transform_point(const System &source_srs, const System &target_srs, const P point) {
    ALP_TRY(const Transform transform, get_transform(source_srs, target_srs));
    return transform.apply(point);
}

// Maps every point of `points` from `source_srs` into `target_srs`.
// Fails if GDAL has no transform between them or one of the points has no image there.
template <Point P>
[[nodiscard]]
inline alp::Result<std::vector<P>, TransformError> transform_points(const System &source_srs, const System &target_srs, std::vector<P> points) {
    ALP_TRY(const Transform transform, get_transform(source_srs, target_srs));
    ALP_TRY(transform.apply_inplace(points));
    return points;
}

}
