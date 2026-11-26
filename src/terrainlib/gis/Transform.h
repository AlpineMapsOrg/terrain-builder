/*****************************************************************************
 * AlpineMaps.org
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

#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <functional>
#include <memory>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include <cpl_port.h>
#include <fmt/format.h>
#include <glm/glm.hpp>
#include <libassert/assert.hpp>

#include "containers/HybridVector.h"
#include "enumerate.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "gis/CplError.h"

namespace gis {

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

} // namespace detail

// Maps coordinate arrays in place between the spaces it names, forwards or, if `inverse` is set, backwards.
// `zs` is null for 2D points, and the result is whether the mapping could be applied at all.
template <typename T>
concept Transformer = std::copyable<T> && requires(const T &transformer, const bool inverse, const size_t count, double *coordinates, int *successes) {
    { transformer(inverse, count, coordinates, coordinates, coordinates, successes) } -> std::same_as<bool>;
    { transformer.source_name() } -> fmt::formattable;
    { transformer.target_name() } -> fmt::formattable;
};

// A point that a transform cannot map.
struct UntransformablePoint : alp::MessageCode<"a point cannot be transformed"> {};

// Which points of a range a transform mapped.
class [[nodiscard]] SuccessMask {
public:
    // Whether every point was mapped.
    explicit operator bool() const {
        return std::ranges::all_of(this->_successes, std::identity());
    }

    // Whether the point at `index` was mapped.
    [[nodiscard]]
    bool is_mapped(const size_t index) const {
        ASSERT(index < this->_successes.size());
        return this->_successes[index] != FALSE;
    }

    // Fails if a point was not mapped, or, if `partial` is set, only if there are points and none was mapped.
    [[nodiscard]]
    alp::Result<void, UntransformablePoint> to_result(const bool partial) const {
        if (partial) {
            if (!this->_successes.empty() && std::ranges::none_of(this->_successes, std::identity())) {
                return alp::make_error(UntransformablePoint{}, "no point has an image");
            }
            return {};
        }
        const auto unmapped = std::ranges::find(this->_successes, FALSE);
        if (unmapped != this->_successes.end()) {
            const auto index = std::ranges::distance(this->_successes.begin(), unmapped);
            const std::string context = fmt::format("point {} has no image", index);
            return alp::make_error(UntransformablePoint{}, context);
        }
        return {};
    }

private:
    template <Transformer T>
    friend class BasicTransform;

    // The mask of the flags GDAL set in `successes`.
    explicit SuccessMask(detail::SuccessBuffer successes) : _successes(std::move(successes)) {}

    detail::SuccessBuffer _successes;
};

// The signature of GDAL's transformer functions, GDALTransformerFunc.
using TransformerFunction = int (*)(void *transformer, int dst_to_src, int count, double *xs, double *ys, double *zs, int *successes);

namespace detail {

// GDAL's transformer function with the argument it receives, between two named spaces.
class ErasedTransformer {
public:
    ErasedTransformer(TransformerFunction function, std::shared_ptr<void> argument, std::string source_name, std::string target_name)
        : _function(function), _argument(std::move(argument)), _source_name(std::move(source_name)), _target_name(std::move(target_name)) {}

    // Maps coordinate arrays in place with GDAL's transformer function, forwards or, if `inverse` is set, backwards.
    bool operator()(const bool inverse, const size_t count, double *xs, double *ys, double *zs, int *successes) const {
        // TODO: switch to a checked cast
        ASSERT(std::in_range<int>(count));
        return this->_function(this->_argument.get(), inverse, static_cast<int>(count), xs, ys, zs, successes) != FALSE;
    }

    std::string_view source_name() const {
        return this->_source_name;
    }

    std::string_view target_name() const {
        return this->_target_name;
    }

private:
    TransformerFunction _function;
    std::shared_ptr<void> _argument;
    std::string _source_name;
    std::string _target_name;
};

// Runs the `T` it receives as its argument, as GDAL's transformer function.
template <Transformer T>
int run_transformer(void *transformer, const int dst_to_src, const int count, double *xs, double *ys, double *zs, int *successes) {
    const T &typed = *static_cast<const T *>(transformer);
    return typed(dst_to_src != FALSE, count, xs, ys, zs, successes) ? TRUE : FALSE;
}

} // namespace detail

// A mapping of points from one space into another by `T`.
// Must only be used on the thread that created it.
template <Transformer T>
class BasicTransform {
public:
    // A transform that runs `transformer`.
    explicit BasicTransform(T transformer) : _transformer(std::move(transformer)) {}

    // Creates the inverse of this transform.
    [[nodiscard]]
    BasicTransform inverse() const {
        BasicTransform inverted = *this;
        inverted._is_inverse = !this->_is_inverse;
        return inverted;
    }

    // Maps a point to its image.
    // Fails if it has no image.
    template <Point P>
    [[nodiscard]]
    alp::Result<P, UntransformablePoint> apply(P point) const {
        std::span<P, 1> points(&point, 1);
        ALP_TRY(this->apply_inplace(&points));
        return point;
    }

    // Replaces every point of a range with its image.
    // Fails if one of them has no image.
    template <WritablePointRange Points>
    [[nodiscard]]
    alp::Result<void, UntransformablePoint> apply_inplace(Points *points) const {
        return this->apply_into(*points, points);
    }

    // Maps every point of `points` and writes the results to `out`, which must hold as many points.
    // Fails if one of them has no image.
    template <PointRange Points, WritablePointRange Out>
        requires(std::ranges::range_value_t<Out>::length() <= std::ranges::range_value_t<Points>::length())
    [[nodiscard]]
    alp::Result<void, UntransformablePoint> apply_into(const Points &points, Out *out) const {
        ALP_TRY(const SuccessMask mask, this->apply_into_partial(points, out));
        ALP_TRY_CONTEXT(mask.to_result(false), this->format_context());
        return {};
    }

    // Maps every point of `points` that has an image, and reports which ones did.
    // Fails only if the transform cannot be applied at all, or fails for every point.
    template <WritablePointRange Points>
    [[nodiscard]]
    alp::Result<SuccessMask, UntransformablePoint> apply_inplace_partial(Points *points) const {
        return this->apply_into_partial(*points, points);
    }

    // Maps every point of `points` that has an image, writes the results to `out`, which must hold as many points, and reports which ones did.
    // Fails only if the transform cannot be applied at all, or fails for every point.
    template <PointRange Points, WritablePointRange Out>
        requires(std::ranges::range_value_t<Out>::length() <= std::ranges::range_value_t<Points>::length())
    [[nodiscard]]
    alp::Result<SuccessMask, UntransformablePoint> apply_into_partial(const Points &points, Out *out) const {
        using OutPoint = std::ranges::range_value_t<Out>;
        constexpr bool is_source_3d = (std::ranges::range_value_t<Points>::length() == 3);
        constexpr bool is_target_3d = (OutPoint::length() == 3);

        const size_t point_count = std::ranges::size(points);
        ASSERT(std::ranges::size(*out) == point_count);
        if (point_count == 0) {
            // Early return since GDAL reports an empty range as a failure.
            return SuccessMask(detail::SuccessBuffer());
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
        // 2D points pass no heights, and an empty buffer still has a non-null data pointer.
        double *zs_data = is_source_3d ? zs.data() : nullptr;
        auto [success, gdal_error] = capture_error([&] {
            return this->_transformer(this->_is_inverse, point_count, xs.data(), ys.data(), zs_data, successes.data());
        });
        SuccessMask mask(std::move(successes));
        // OGR reports a failure as soon as one point fails, so only a failure without any mapped point fails the call.
        if (!success) {
            const auto mask_result = mask.to_result(true);
            if (!mask_result) {
                return alp::make_error(mask_result.error().code(), this->format_context(), std::move(gdal_error));
            }
        }

        for (auto [i, out_point] : enumerate(*out)) {
            if constexpr (is_target_3d) {
                out_point = OutPoint(xs[i], ys[i], zs[i]);
            } else {
                out_point = OutPoint(xs[i], ys[i]);
            }
        }
        return mask;
    }

    // Converts this into a transform of any kind.
    explicit operator BasicTransform<detail::ErasedTransformer>() const
        requires(!std::same_as<T, detail::ErasedTransformer>)
    {
        std::shared_ptr<void> argument = std::make_shared<T>(this->_transformer);
        std::string source_name = fmt::format("{}", this->_transformer.source_name());
        std::string target_name = fmt::format("{}", this->_transformer.target_name());
        detail::ErasedTransformer transformer(&detail::run_transformer<T>, std::move(argument), std::move(source_name), std::move(target_name));
        const BasicTransform<detail::ErasedTransformer> transform(std::move(transformer));
        if (this->_is_inverse) {
            return transform.inverse();
        } else {
            return transform;
        }
    }

protected:
    // The transformer this maps by.
    [[nodiscard]]
    const T &transformer() const {
        return this->_transformer;
    }

    // Whether this maps backwards.
    [[nodiscard]]
    bool is_inverse() const {
        return this->_is_inverse;
    }

private:
    // Names the spaces `transform` maps between, in its direction.
    friend std::string format_as(const BasicTransform &transform) {
        const T &transformer = transform._transformer;
        if (transform._is_inverse) {
            return fmt::format("{} -> {}", transformer.target_name(), transformer.source_name());
        } else {
            return fmt::format("{} -> {}", transformer.source_name(), transformer.target_name());
        }
    }

    // Formats the context of this transform's failures.
    [[nodiscard]]
    std::string format_context() const {
        return fmt::format("applying the transform {}", *this);
    }

    T _transformer;
    bool _is_inverse = false;
};

// A transform of any kind, run as GDAL's transformer function.
using Transform = BasicTransform<detail::ErasedTransformer>;

// Creates a transform by GDAL's transformer `function`, which receives `argument`, from the space named `source_name` into the one named `target_name`.
[[nodiscard]]
inline Transform make_transform(TransformerFunction function, std::shared_ptr<void> argument, std::string source_name, std::string target_name) {
    detail::ErasedTransformer transformer(function, std::move(argument), std::move(source_name), std::move(target_name));
    return Transform(std::move(transformer));
}

} // namespace gis
