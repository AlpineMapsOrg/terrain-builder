#include <catch2/catch.hpp>

#include "ctb/Grid.hpp"
#include "Tiler.h"
#include "MetaDataGenerator.h"

TEST_CASE("layer.json writer") {
  const auto metadata = MetaDataGenerator::make(ATB_TEST_DATA_DIR "/austria/at_mgi.tif", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::Tms);
}
