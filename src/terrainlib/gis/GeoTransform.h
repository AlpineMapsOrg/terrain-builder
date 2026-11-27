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

#include <array>
#include <string_view>

#include <glm/vec2.hpp>

#include "error/Result.h"

namespace gis {

// Why GDAL's coefficients do not form a geotransform.
enum class InvalidGeoTransform {
    NotFinite,
    NotInvertible
};

inline std::string_view format_as(const InvalidGeoTransform error) {
    switch (error) {
    case InvalidGeoTransform::NotFinite:
        return "a coefficient is not finite";
    case InvalidGeoTransform::NotInvertible:
        return "the coefficients cannot be inverted";
    default:
        return "an unknown geotransform failure";
    }
}

// The affine mapping from pixels into a system, which GDAL calls the geotransform.
class GeoTransform {
public:
    // Creates the geotransform with GDAL's six coefficients.
    // Fails if one of them is not finite or they cannot be inverted.
    [[nodiscard]]
    static alp::Result<GeoTransform, InvalidGeoTransform> from_coefficients(const std::array<double, 6> &coefficients);

    bool operator==(const GeoTransform &) const = default;

    // Maps a pixel to its point in the system.
    [[nodiscard]]
    glm::dvec2 apply(glm::dvec2 pixel) const;

    // Maps a point in the system to its pixel.
    [[nodiscard]]
    glm::dvec2 apply_inverse(glm::dvec2 point) const;

    // Whether the transform has neither shear nor rotation.
    [[nodiscard]]
    bool is_axis_aligned() const;

    // The extent of one pixel along x and y in the system, negative where an axis runs against the system's.
    [[nodiscard]]
    glm::dvec2 pixel_size() const;

private:
    GeoTransform(const std::array<double, 6> &forward, const std::array<double, 6> &inverse) : _forward(forward), _inverse(inverse) {}

    std::array<double, 6> _forward;
    std::array<double, 6> _inverse;
};

} // namespace gis
