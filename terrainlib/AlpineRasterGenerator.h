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
#include "ctb/types.hpp"

class Dataset;
using DatasetPtr = std::shared_ptr<Dataset>;

class AlpineRasterGenerator
{
public:
  AlpineRasterGenerator(const std::string& output_data_path, const std::string& input_data_path, const ctb::Grid& grid, const Tiler& tiler);
  [[nodiscard]] static AlpineRasterGenerator make(const std::string& output_data_path, const std::string& input_data_path, ctb::Grid::Srs srs, Tiler::Scheme tiling_scheme, Tiler::Border border);

  [[nodiscard]] static glm::u8vec3 convert(float height);
  [[nodiscard]] static RgbImage convertHeights(const HeightData& heights);
  void write(const ctb::TilePoint& tilepoint, ctb::i_zoom zoom, const HeightData& heights) const;
  [[nodiscard]] std::vector<Tile> listTiles() const;
  [[nodiscard]] std::vector<Tile> listTiles(ctb::i_zoom max_zoom) const;
  void process() const;
  void process(ctb::i_zoom max_zoom) const;

protected:
  ctb::i_zoom maxZoom() const;


private:
  std::string m_output_data_path;
  std::string m_input_data_path;
  ctb::Grid m_grid;
  Tiler m_tiler;

};

#endif // ALPINERASTERGENERATOR_H
