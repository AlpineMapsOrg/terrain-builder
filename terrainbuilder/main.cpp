#include <glm/glm.hpp>

#include "alpine_raster.h"
#include "cesium_tin_terra.h"
#include "ctb/Grid.hpp"
#include "Tiler.h"

int main() {
//  const auto generator = alpine_raster::make_generator("./test_tiles/", "/home/madam/valtava/raw/Oe_2020/OeRect_01m_gs_31287.img", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::SlippyMap, Tiler::Border::No);
//  generator.process({16, 16});
  const auto generator = cesium_tin_terra::make_generator("./test_tiles/", "/home/madam/valtava/raw/Oe_2020/OeRect_01m_gs_31287.img", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::Tms, Tiler::Border::Yes);
  generator.process({0, 6}, true);

  return 0;
}
