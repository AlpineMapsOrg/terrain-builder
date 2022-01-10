#include <glm/glm.hpp>

#include "AlpineRasterGenerator.h"
#include "ctb/Grid.hpp"
#include "Tiler.h"

int main() {
  const auto generator = AlpineRasterGenerator::make("./test_tiles/", "/home/madam/valtava/raw/Oe_2020/OeRect_01m_gs_31287.img", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::SlippyMap, Tiler::Border::No);
  generator.process({16, 16});

  return 0;
}
