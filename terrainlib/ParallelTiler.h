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

#ifndef PARALLELTILER_H
#define PARALLELTILER_H

#include "ctb/Grid.hpp"
#include "ctb/types.hpp"

#include "Tile.h"

class ParallelTiler
{
public:
  enum class Border {
    Yes = 1, No = 0
  };

  // The difference between TMS and slippyMap is whether y starts counting from the bottom (south) or top (north).
  // https://www.maptiler.com/google-maps-coordinates-tile-bounds-projection/#1/-16.88/79.02
  //
  enum class Scheme {
    Tms,      // southern most tile is y = 0
    SlippyMap // aka Google, XYZ, webmap tiles; northern most tile is y = 0
  };

  ParallelTiler(const ctb::Grid& grid, const ctb::CRSBounds& bounds, Border border, Scheme scheme);

  [[nodiscard]] std::vector<Tile> generateTiles(ctb::i_zoom zoom_level) const;
  [[nodiscard]] std::vector<Tile> generateTiles(const std::pair<ctb::i_zoom, ctb::i_zoom>& zoom_range) const;
  [[nodiscard]] Scheme scheme() const;

  [[nodiscard]] ctb::TileCoordinate southWestTile(ctb::i_zoom zoom_level) const;
  [[nodiscard]] ctb::TileCoordinate northEastTile(ctb::i_zoom zoom_level) const;

  const ctb::CRSBounds& bounds() const;
  void setBounds(const ctb::CRSBounds& newBounds);

private:
  [[nodiscard]] ctb::TileCoordinate convertToTilerScheme(const ctb::TileCoordinate&, ctb::i_tile n_y_tiles) const;
  [[nodiscard]] ctb::i_tile n_y_tiles(ctb::i_zoom zoom_level) const;

  const ctb::Grid m_grid;
  ctb::CRSBounds m_bounds;
  const Border m_border_south_east;
  const Scheme m_scheme;
};

#endif // PARALLELTILER_H
