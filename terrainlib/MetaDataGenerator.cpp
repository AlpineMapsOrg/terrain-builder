#include "MetaDataGenerator.h"
#include "Exception.h"
#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "ctb/types.hpp"

namespace {
ctb::Grid srs2grid(ctb::Grid::Srs srs) {
  switch (srs) {
  case ctb::Grid::Srs::SphericalMercator:
    return ctb::GlobalMercator();
  case ctb::Grid::Srs::WGS84:
    return ctb::GlobalGeodetic(256);
  }
  throw Exception("Not implemented!");
}
}

MetaDataGenerator::MetaDataGenerator(const DatasetPtr& dataset, const ctb::Grid& grid, const Tiler& tiler) : m_dataset(dataset), m_grid(grid), m_tiler(tiler)
{

}

MetaDataGenerator MetaDataGenerator::make(const std::string& input_data_path, ctb::Grid::Srs srs, Tiler::Scheme tiling_scheme)
{
  auto dataset = Dataset::make_shared(input_data_path);
  auto grid = srs2grid(srs);
  auto tiler = Tiler(grid, dataset->bounds(grid.getSRS()), Tiler::Border::No, tiling_scheme);   // border does not matter for the metadata, no or true would both work.
  return MetaDataGenerator(dataset, grid, tiler);
}

const ctb::Grid& MetaDataGenerator::grid() const
{
  return m_grid;
}

const Tiler& MetaDataGenerator::tiler() const
{
  return m_tiler;
}

std::vector<ctb::TileBounds> MetaDataGenerator::availableTiles() const
{
  std::vector<ctb::TileBounds> list;
  const auto max_zoom = m_grid.zoomForResolution(m_dataset->gridResolution(m_grid.getSRS()));
  for (ctb::i_zoom i = 0; i < max_zoom; ++i) {
    const auto sw = m_tiler.southWestTile(i);
    const auto ne = m_tiler.northEastTile(i);
    list.emplace_back(sw, ne);
  }
  return list;
}

const DatasetPtr& MetaDataGenerator::dataset() const
{
  return m_dataset;
}
