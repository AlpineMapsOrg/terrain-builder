#include "Tiler.h"

#include "Exception.h"


Tiler::Tiler(const ctb::Grid& grid, const ctb::CRSBounds& bounds, Border border) :
    m_grid(grid), m_bounds(bounds),
    m_border_south_east(border)
{

}

std::vector<Tile> Tiler::generateTiles(ctb::i_zoom zoom_level) const
{

  const auto sw = m_grid.crsToTile(m_bounds.getLowerLeft(), zoom_level);
  const auto epsilon = m_grid.resolution(zoom_level) / 100;
  const auto ne = m_grid.crsToTile(m_bounds.getUpperRight() - epsilon, zoom_level);

  std::vector<Tile> tiles;
  for (auto ty = sw.y; ty <= ne.y; ++ty) {
    for (auto tx = sw.x; tx <= ne.x; ++tx) {
      ctb::TilePoint point = {tx, ty};
      ctb::CRSBounds srs_bounds = m_grid.srsBounds(ctb::TileCoordinate(zoom_level, tx, ty), m_border_south_east == Border::Yes);
      srs_bounds.clampBy(m_grid.getExtent());
      ctb::i_tile grid_size = m_grid.tileSize();
      ctb::i_tile tile_size = grid_size + unsigned(m_border_south_east);
      tiles.emplace_back(point, zoom_level, srs_bounds, grid_size, tile_size);

      if (tiles.size() >= 100'000'000)
        // think about creating an on the fly tile generator. storing so many tiles takes a lot of memory.
        throw Exception("Setting the zoom level so higher is probably not a good idea. This would generate more than 100 million tiles. "
                        "I'm aborting. If you really need this, then that means that the future is bright. But you'll have to edit the code..\n"
                        "      .   .     \n     / \\_/ \\    \n    | O  O |   \n    | ~V~  |  _\n     ~_ _ /  // \n    /     \\ //  \n"
                        "    |  ||  |/  \n    | /  \\ |   \n    ||    ||   \n               \n");
    }
  }

  return tiles;
}
