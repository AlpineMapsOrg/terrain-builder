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

#include "gis/srs/geographic.h"

#include <numbers>
#include <optional>
#include <vector>

#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "error/Context.h"
#include "error/utils.h"
#include "geometry/geometry.h"
#include "gis/srs/Kind.h"
#include "gis/srs/Unit.h"
#include "gis/srs/classify.h"

namespace gis::srs {

alp::Result<std::vector<radix::tile::SrsBounds>, WrapLongitudesError> wrap_longitudes(const System &reference, const radix::tile::SrsBounds &bounds, const glm::dvec2 &centre) {
    const auto context = alp::make_context("wrapping longitudes in {}", reference);
    if (reference.kind() != Kind::Geographic) {
        return alp::make_error(NotGeographic{}, context);
    }

    const std::optional<LongitudeLatitudeAxes> longitude_latitude_axes_opt = longitude_latitude_axes(reference);
    if (!longitude_latitude_axes_opt.has_value()) {
        return alp::make_error(UnsupportedAxisOrder{}, context);
    }
    const IndexedAxis longitude = longitude_latitude_axes_opt->longitude;
    // The bounds are horizontal, so their longitude has to be one of their two dimensions.
    if (longitude.index >= 2) {
        return alp::make_error(UnsupportedAxisOrder{}, context);
    }

    const glm::length_t dimension = static_cast<glm::length_t>(longitude.index);
    glm::bvec2 longitude_only(false);
    longitude_only[dimension] = true;
    glm::dvec2 period(0);
    period[dimension] = convert(2 * std::numbers::pi, units::radian, longitude.axis.unit);
    const std::vector<radix::geometry::Aabb<2, double>> wrapped = geometry::wrap_into_periods(bounds, longitude_only, centre, period);
    return std::vector<radix::tile::SrsBounds>(wrapped.begin(), wrapped.end());
}

}
