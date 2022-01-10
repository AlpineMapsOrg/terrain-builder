#ifndef ALPINERASTERGENERATOR_H
#define ALPINERASTERGENERATOR_H

#include <memory>
#include <string>

#include <glm/glm.hpp>
#include <vector>

#include "Image.h"
#include "ctb/Grid.hpp"
#include "Tile.h"
#include "Tiler.h"

class Dataset;
using DatasetPtr = std::shared_ptr<Dataset>;

class AlpineRasterGenerator
{
public:
  AlpineRasterGenerator(const std::string& output_data_path, DatasetPtr dataset, const ctb::Grid& grid, const Tiler& tiler);
  [[nodiscard]] static AlpineRasterGenerator make(const std::string& output_data_path, const std::string& input_data_path, ctb::Grid::Srs srs, Tiler::Scheme tiling_scheme, Tiler::Border border);

  [[nodiscard]] static glm::u8vec3 convert(float height);
  [[nodiscard]] static RgbImage convertHeights(const HeightData& heights);
  void write(const ctb::TilePoint& tilepoint, ctb::i_zoom zoom, const HeightData& heights) const;
  [[nodiscard]] std::vector<Tile> listTiles() const;
  void process() const;


private:
  std::string m_output_data_path;
  DatasetPtr m_dataset;
  ctb::Grid m_grid;
  Tiler m_tiler;

};

#endif // ALPINERASTERGENERATOR_H
