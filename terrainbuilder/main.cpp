#include <glm/glm.hpp>

#include "alpine_raster.h"
#include "ctb/Grid.hpp"
#include "Tiler.h"

int main() {
  const auto generator = alpine_raster::make_generator("./test_tiles/", "/home/madam/valtava/raw/Oe_2020/OeRect_01m_gs_31287.img", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::SlippyMap, Tiler::Border::No);
  generator.process({16, 16});

  return 0;
}
