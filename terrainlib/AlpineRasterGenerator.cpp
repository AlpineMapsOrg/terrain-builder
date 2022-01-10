#include "AlpineRasterGenerator.h"

#include <algorithm>

#include <filesystem>
#include <fmt/core.h>
#include <iterator>
#include <memory>

#include "Dataset.h"
#include "DatasetReader.h"
#include "Exception.h"
#include "Image.h"
#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "ctb/types.hpp"

AlpineRasterGenerator::AlpineRasterGenerator(const std::string& output_data_path, DatasetPtr dataset, const ctb::Grid& grid, const Tiler& tiler) :
    m_output_data_path(output_data_path), m_dataset(dataset), m_grid(grid), m_tiler(tiler)
{

}

AlpineRasterGenerator AlpineRasterGenerator::make(const std::string& output_data_path, const std::string& input_data_path, ctb::Grid::Srs srs, Tiler::Scheme tiling_scheme, Tiler::Border border)
{
  const auto dataset = Dataset::make_shared(input_data_path);
  ctb::Grid grid = ctb::GlobalGeodetic(256);
  if (srs == ctb::Grid::Srs::SphericalMercator)
    grid = ctb::GlobalMercator(256);

  return {output_data_path, dataset, grid, Tiler(grid, dataset->bounds(grid.getSRS()), border, tiling_scheme)};
}

glm::u8vec3 AlpineRasterGenerator::convert(float height)
{
  const auto r = std::clamp(int(height / 32.0f), 0, 255);
  const auto g = std::clamp(int(std::fmod(height, 32.0f) * 8), 0, 255);

  return {glm::u8(r), glm::u8(g), 0};
}

RgbImage AlpineRasterGenerator::convertHeights(const HeightData& heights)
{
  return image::transformImage(heights, AlpineRasterGenerator::convert);
}

void AlpineRasterGenerator::write(const ctb::TilePoint& tilepoint, ctb::i_zoom zoom, const HeightData& heights) const
{
  const auto dir_path = fmt::format("{}/{}/{}", m_output_data_path, zoom, tilepoint.x);
  const auto file_path = fmt::format("{}/{}.png", dir_path, tilepoint.y);
  std::filesystem::create_directories(dir_path);
  image::saveImageAsPng(convertHeights(heights), file_path);
}

std::vector<Tile> AlpineRasterGenerator::listTiles() const
{
  return listTiles(maxZoom());
}

std::vector<Tile> AlpineRasterGenerator::listTiles(ctb::i_zoom max_zoom) const
{
  std::vector<Tile> tiles;
  for (ctb::i_zoom i = 0; i < max_zoom; ++i) {
    auto zoom_level_tiles = m_tiler.generateTiles(i);
    std::move(zoom_level_tiles.begin(), zoom_level_tiles.end(), std::back_inserter(tiles));
  }
  return tiles;
}

void AlpineRasterGenerator::process() const
{
  return process(maxZoom());
}

void AlpineRasterGenerator::process(ctb::i_zoom max_zoom) const
{
  DatasetReader reader(m_dataset, m_grid.getSRS(), 1);
  const auto tiles = listTiles(max_zoom);
  const auto fun = [&reader, this](const Tile& tile) {
    const auto heights = reader.readWithOverviews(tile.srsBounds, tile.tileSize, tile.tileSize);
    write(tile.point, tile.zoom, heights);
  };
  std::for_each(tiles.begin(), tiles.end(), fun);
}

ctb::i_zoom AlpineRasterGenerator::maxZoom() const
{
  return m_grid.zoomForResolution(m_dataset->gridResolution(m_grid.getSRS()));
}
