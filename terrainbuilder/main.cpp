#include <glm/glm.hpp>

#include "alpine_raster.h"
#include "cesium_tin_terra.h"
#include "ctb/Grid.hpp"
#include "Tiler.h"

int main() {
//  const auto generator = alpine_raster::make_generator("./test_tiles/", "/home/madam/valtava/raw/Oe_2020/OeRect_01m_gs_31287.img", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::SlippyMap, Tiler::Border::No);
//  generator.process({16, 16});
  const auto generator = cesium_tin_terra::make_generator("/home/madam/valtava/raw/Oe_2020/OeRect_01m_gs_31287.img", "/home/madam/Documents/work/tuw/alpinemaps/www/tiles/ctb_test/", ctb::Grid::Srs::WGS84, Tiler::Scheme::Tms, Tiler::Border::No);
  generator.process({0, 5}, true, true);
  generator.process({6, 10}, true, false);

  return 0;
}
