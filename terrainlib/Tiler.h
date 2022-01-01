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

public:
  Tiler(const ctb::Grid& grid, const ctb::CRSBounds& bounds, Border border);

  std::vector<Tile> generateTiles(ctb::i_zoom zoom_level) const;

private:
  const ctb::Grid m_grid;
  const ctb::CRSBounds m_bounds;
  const Border m_border_south_east;
};

#endif // TILER_H
