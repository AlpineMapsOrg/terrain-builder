#ifndef METADATAGENERATOR_H
#define METADATAGENERATOR_H

#include <string>
#include <vector>

#include "Dataset.h"
#include "ctb/Grid.hpp"
#include "Tiler.h"
#include "ctb/types.hpp"


class MetaDataGenerator
{
public:
  MetaDataGenerator(const DatasetPtr& input_data_path, const ctb::Grid& grid, const Tiler& tiler);
  [[nodiscard]] static MetaDataGenerator make(const std::string& input_data_path, ctb::Grid::Srs srs, Tiler::Scheme tiling_scheme);
  [[nodiscard]] const ctb::Grid& grid() const;
  [[nodiscard]] const Tiler& tiler() const;

  [[nodiscard]] std::vector<ctb::TileBounds> availableTiles() const;

private:
  DatasetPtr m_dataset;
  ctb::Grid m_grid;
  Tiler m_tiler;
};

#endif // METADATAGENERATOR_H
