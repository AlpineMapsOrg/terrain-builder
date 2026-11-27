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

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cpl_port.h>
#include <fmt/format.h>
#include <ogr_spatialref.h>

#include "error/Error.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "gis/CplError.h"
#include "gis/Transform.h"
#include "gis/gdal.h"
#include "gis/srs/System.h"

namespace gis::srs {

// GDAL provides no transform from one spatial reference to the other.
struct UnsupportedTransform : alp::MessageCode<"GDAL provides no transform between the two spatial references"> {};

// An error from transforming between two spatial references.
using TransformError = alp::Error<UnsupportedTransform, UntransformablePoint>;

namespace detail {

// The name of `system`, or "unknown srs" if there is none.
inline std::string format_system_name(const OGRSpatialReference *system) {
    return system != nullptr ? fmt::format("{}", System(system)) : "unknown srs";
}

// Both directions of one OGR transformation, of which the inverse is created on first use.
class CoordinateTransformer {
public:
    explicit CoordinateTransformer(std::shared_ptr<OGRCoordinateTransformation> forward) : _forward(std::move(forward)) {}

    // Maps coordinate arrays in place with the forward transformation, or with the inverse if `inverse` is set.
    bool operator()(const bool inverse, const size_t count, double *xs, double *ys, double *zs, int *successes) const {
        OGRCoordinateTransformation *transformation = inverse ? this->inverse() : this->forward();
        if (transformation == nullptr) {
            return false;
        }
        return transformation->Transform(count, xs, ys, zs, successes) != FALSE;
    }

    // The system this maps from, or "unknown srs" if GDAL's transformation has none.
    std::string source_name() const {
        return format_system_name(this->_forward->GetSourceCS());
    }

    // The system this maps into, or "unknown srs" if GDAL's transformation has none.
    std::string target_name() const {
        return format_system_name(this->_forward->GetTargetCS());
    }

    // The forward transformation.
    OGRCoordinateTransformation *forward() const {
        return this->_forward.get();
    }

    // The inverse transformation, created on first use, or null if GDAL has none.
    OGRCoordinateTransformation *inverse() const {
        if (!this->_inverse) {
            this->_inverse.reset(this->_forward->GetInverse());
        }
        return this->_inverse.get();
    }

private:
    std::shared_ptr<OGRCoordinateTransformation> _forward;
    // The call operator is const, so creating the inverse on first use needs mutable.
    // This is sound because a transform is only used on one thread.
    mutable std::shared_ptr<OGRCoordinateTransformation> _inverse;
};

} // namespace detail

// A transform from one spatial reference to another, which must only be used on the thread it was created on.
class Transform : public BasicTransform<detail::CoordinateTransformer> {
public:
    // A transform that shares ownership of the GDAL transformation.
    explicit Transform(std::shared_ptr<OGRCoordinateTransformation> gdal_transform)
        : BasicTransform(detail::CoordinateTransformer(std::move(gdal_transform))) {}

    // A transform that refers to the GDAL transformation, which must outlive it.
    explicit Transform(OGRCoordinateTransformation *gdal_transform)
        : Transform(std::shared_ptr<OGRCoordinateTransformation>(std::shared_ptr<void>(), gdal_transform)) {}

    // The GDAL transformation this wraps.
    [[nodiscard]]
    OGRCoordinateTransformation *as_gdal() const {
        return this->is_inverse() ? this->transformer().inverse() : this->transformer().forward();
    }

    // Creates the inverse of this transform.
    // Fails if GDAL has no transform in that direction.
    [[nodiscard]]
    alp::Result<Transform, UnsupportedTransform> inverse() const {
        auto [gdal_inverse, gdal_error] = capture_error([&] {
            return this->transformer().inverse();
        });
        if (gdal_inverse == nullptr) {
            const std::string context = fmt::format("inverting the transform {}", *this);
            return alp::make_error(UnsupportedTransform{}, context, std::move(gdal_error));
        }
        return Transform(this->BasicTransform::inverse());
    }

private:
    explicit Transform(BasicTransform transform) : BasicTransform(std::move(transform)) {}
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
inline alp::Result<Transform, UnsupportedTransform> create_transform(const System &source_srs, const System &target_srs) {
    gdal::initialize_once();
    auto [created, gdal_error] = capture_error([&] {
        return OGRCreateCoordinateTransformation(&source_srs.as_gdal(), &target_srs.as_gdal());
    });
    std::shared_ptr<OGRCoordinateTransformation> gdal_transform(created);
    if (!gdal_transform) {
        const std::string context = fmt::format("creating a transform from {} to {}", source_srs, target_srs);
        return alp::make_error(UnsupportedTransform{}, context, std::move(gdal_error));
    }
    return Transform(std::move(gdal_transform));
}

// Gives the calling thread its transform from `source_srs` to `target_srs`.
// Fails if GDAL has no transform between them.
[[nodiscard]]
inline alp::Result<Transform, UnsupportedTransform> get_transform(const System &source_srs, const System &target_srs) {
    thread_local std::vector<detail::TransformCacheEntry> cache;

    // Check forward
    for (auto &entry : cache) {
        if (entry.source.is_same(source_srs) && entry.target.is_same(target_srs)) {
            return entry.transform;
        }
    }

    // Check inverse
    for (auto &entry : cache) {
        if (entry.source.is_same(target_srs) && entry.target.is_same(source_srs)) {
            return entry.transform.inverse();
        }
    }

    // Not found
    ALP_TRY(const Transform transform, create_transform(source_srs, target_srs));
    detail::TransformCacheEntry entry{source_srs.clone(), target_srs.clone(), transform};
    cache.push_back(std::move(entry));
    return transform;
}

// Maps a point from one spatial reference into another.
// Fails if GDAL has no transform between them or the point has no image there.
template <Point P>
[[nodiscard]]
inline alp::Result<P, TransformError> transform_point(const System &source_srs, const System &target_srs, const P point) {
    ALP_TRY(const Transform transform, get_transform(source_srs, target_srs));
    return transform.apply(point);
}

// Maps every point of a list from one spatial reference into another.
// Fails if GDAL has no transform between them or one of the points has no image there.
template <Point P>
[[nodiscard]]
inline alp::Result<std::vector<P>, TransformError> transform_points(const System &source_srs, const System &target_srs, std::vector<P> points) {
    ALP_TRY(const Transform transform, get_transform(source_srs, target_srs));
    ALP_TRY(transform.apply_inplace(&points));
    return points;
}

}
