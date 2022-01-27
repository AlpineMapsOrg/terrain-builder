/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 alpinemaps.org
 * Copyright (C) 2022 Adam Celarek <family name at cg tuwien ac at>
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

#ifndef ALGORITHMS_RASTER_TRIANGLE_SCANLINE_H
#define ALGORITHMS_RASTER_TRIANGLE_SCANLINE_H

#include <glm/common.hpp>
#include <glm/fwd.hpp>
#include <glm/glm.hpp>
#include "tntn/Raster.h"
#include "primitives.h"

namespace raster {

template <typename T, typename Lambda>
void triangle_scanline(const tntn::Raster<T>& raster, const glm::uvec2& a, const glm::uvec2& b, const glm::uvec2& c, const Lambda& fun) {
  const auto min = glm::min(glm::min(a, b), c);
  const auto max = glm::max(glm::max(a, b), c);
  for (auto x = min.x; x < max.x; ++x) {
    for (auto y = min.y; y < max.y; ++y) {
      const auto coord = glm::uvec2(x, y);
      if (primitives::inside(coord, a, b, c))
        fun(coord, raster.value(coord.y, coord.x)); // raster is row / column
    }
  }
  if (min.y == 0) {
    // bottom edge of raster is not included in primitives::inside, so check for it and make an extrawurscht.
    const auto walk_bottom_edge = [&](const glm::uvec2& a, const glm::uvec2& b) {
      if ((b - a).y == 0 && a.y == 0) {
        const auto end_x = std::max(a.x, b.x);
        for (auto x = std::min(a.x, b.x); x < end_x; ++x) {
          const auto coord = glm::uvec2(x, 0);
          fun(coord, raster.value(coord.y, coord.x)); // raster is row / column
        }
      }
    };
    walk_bottom_edge(a, b);
    walk_bottom_edge(b, c);
    walk_bottom_edge(c, a);

  }
}
}
#endif
