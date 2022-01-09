#ifndef TILER_H
#define TILER_H

#include "ctb/Grid.hpp"
#include "ctb/types.hpp"

#include "Tile.h"

class Tiler
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

  Tiler(const ctb::Grid& grid, const ctb::CRSBounds& bounds, Border border, Scheme scheme);

  [[nodiscard]] std::vector<Tile> generateTiles(ctb::i_zoom zoom_level) const;
  [[nodiscard]] Scheme scheme() const;

  [[nodiscard]] ctb::TileCoordinate southWestTile(ctb::i_zoom zoom_level) const;
  [[nodiscard]] ctb::TileCoordinate northEastTile(ctb::i_zoom zoom_level) const;

private:
  [[nodiscard]] ctb::TileCoordinate convertToTilerScheme(const ctb::TileCoordinate&, ctb::i_tile n_y_tiles) const;
  [[nodiscard]] ctb::i_tile n_y_tiles(ctb::i_zoom zoom_level) const;

  const ctb::Grid m_grid;
  const ctb::CRSBounds m_bounds;
  const Border m_border_south_east;
  const Scheme m_scheme;
};

#endif // TILER_H
