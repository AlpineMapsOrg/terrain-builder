/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 alpinemaps.org
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

#include <atomic>
#include <memory>
#include <utility>

#include <catch2/catch.hpp>
#include "ParallelTileGenerator.h"


TEST_CASE("parallel tile generator") {
  std::atomic<int> tile_counter = 0;

  class MockTileWriter : public ParallelTileWriterInterface {
    std::atomic<int>* m_tile_counter = nullptr;
  public:
    MockTileWriter(std::atomic<int>* tile_counter) : ParallelTileWriterInterface(Tiler::Border::No), m_tile_counter(tile_counter) {}
    void write(const std::string& base_path, const Tile& tile, const HeightData& heights) const override {
      CHECK(base_path.size() > 0);
      CHECK(tile.gridSize == 256);
      CHECK(heights.width() == 256);
      CHECK(heights.height() == 256);
      REQUIRE(m_tile_counter != nullptr);
      (*m_tile_counter)++;
    }
  };

  const auto generator = ParallelTileGenerator::make(ATB_TEST_DATA_DIR "/austria/at_mgi.tif", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::Tms, std::make_unique<MockTileWriter>(&tile_counter), "./unittest_tiles/");
  generator.process({0, 7});
  CHECK(tile_counter == 27);
}
