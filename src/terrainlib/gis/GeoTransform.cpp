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

#include <algorithm>
#include <cmath>

#include <gdal.h>
#include <gdal_alg.h>

#include "error/utils.h"
#include "gis/GeoTransform.h"

namespace gis {

alp::Result<GeoTransform, InvalidGeoTransform> GeoTransform::from_coefficients(const std::array<double, 6> &coefficients) {
    if (!std::ranges::all_of(coefficients, [](const double coefficient) { return std::isfinite(coefficient); })) {
        return alp::make_error(InvalidGeoTransform::NotFinite);
    }
    std::array<double, 6> inverse;
    if (GDALInvGeoTransform(coefficients.data(), inverse.data()) != TRUE) {
        return alp::make_error(InvalidGeoTransform::NotInvertible);
    }
    return GeoTransform(coefficients, inverse);
}

glm::dvec2 GeoTransform::apply(const glm::dvec2 pixel) const {
    glm::dvec2 point;
    GDALApplyGeoTransform(this->_forward.data(), pixel.x, pixel.y, &point.x, &point.y);
    return point;
}

glm::dvec2 GeoTransform::apply_inverse(const glm::dvec2 point) const {
    glm::dvec2 pixel;
    GDALApplyGeoTransform(this->_inverse.data(), point.x, point.y, &pixel.x, &pixel.y);
    return pixel;
}

bool GeoTransform::is_axis_aligned() const {
    return this->_forward[2] == 0.0 && this->_forward[4] == 0.0;
}

glm::dvec2 GeoTransform::pixel_size() const {
    return glm::dvec2(this->_forward[1], this->_forward[5]);
}

} // namespace gis
