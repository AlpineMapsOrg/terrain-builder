/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
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

#include <glm/glm.hpp>

namespace earth {

// Chimborazo, the summit farthest from the earth's centre
// https://en.wikipedia.org/wiki/Summits_farthest_from_the_Earth%27s_center
inline constexpr double largest_radius = 6384400;
// Litke Deep, the point of the earth's surface closest to its centre
// https://en.wikipedia.org/wiki/Litke_Deep
inline constexpr double smallest_radius = 6351704.3;
// The IUGG mean radius
// https://en.wikipedia.org/wiki/Earth_radius
inline constexpr double radius = 6371008.7714;
inline constexpr glm::dvec2 radius_range{smallest_radius, largest_radius};

// Challenger Deep, 10935 +- 6 m below mean sea level at 95% confidence
// Greenaway et al. 2021, https://doi.org/10.1016/j.dsr.2021.103644
inline constexpr double lowest_elevation = -10941;
// Mount Everest, as surveyed by China and Nepal in 2020
// https://en.wikipedia.org/wiki/Mount_Everest
inline constexpr double highest_elevation = 8848.86;
inline constexpr glm::dvec2 elevation_range{lowest_elevation, highest_elevation};
// The Indian Ocean Geoid Low, the geoid's largest departure from the WGS84 ellipsoid
// EGM96 geoid grid (PROJ's egm96_15.gtx: -107.0 to +85.4 m); https://en.wikipedia.org/wiki/Geoid gives -106 to +85 m
inline constexpr double largest_geoid_undulation = 107;

}
