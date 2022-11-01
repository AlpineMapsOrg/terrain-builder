/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 alpinemaps.org
 * Copyright (C) 2022 Adam Celarek <family name at cg tuwien ac at>
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

#include "MetaDataGenerator.h"
#include "Exception.h"
#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "ctb/types.hpp"

namespace {
ctb::Grid srs2grid(ctb::Grid::Srs srs)
{
    switch (srs) {
    case ctb::Grid::Srs::SphericalMercator:
        return ctb::GlobalMercator();
    case ctb::Grid::Srs::WGS84:
        return ctb::GlobalGeodetic(256);
    }
    throw Exception("Not implemented!");
}
}

MetaDataGenerator::MetaDataGenerator(const DatasetPtr& dataset, const ctb::Grid& grid, const ParallelTiler& tiler)
    : m_dataset(dataset)
    , m_grid(grid)
    , m_tiler(tiler)
{
}

MetaDataGenerator MetaDataGenerator::make(const std::string& input_data_path, ctb::Grid::Srs srs, Tile::Scheme tiling_scheme)
{
    auto dataset = Dataset::make_shared(input_data_path);
    auto grid = srs2grid(srs);
    auto tiler = ParallelTiler(grid, dataset->bounds(grid.getSRS()), Tile::Border::No, tiling_scheme); // border does not matter for the metadata, no or true would both work.
    return MetaDataGenerator(dataset, grid, tiler);
}

const ctb::Grid& MetaDataGenerator::grid() const
{
    return m_grid;
}

const ParallelTiler& MetaDataGenerator::tiler() const
{
    return m_tiler;
}

std::vector<ctb::TileBounds> MetaDataGenerator::availableTiles(unsigned max_zoom) const
{
    std::vector<ctb::TileBounds> list;
    if (max_zoom == unsigned(-1))
        max_zoom = m_grid.zoomForResolution(m_dataset->gridResolution(m_grid.getSRS()));
    for (ctb::i_zoom i = 0; i < max_zoom; ++i) {
        const auto sw = m_tiler.southWestTile(i).coords;
        const auto ne = m_tiler.northEastTile(i).coords;
        list.emplace_back(sw, ne);
    }
    return list;
}

const DatasetPtr& MetaDataGenerator::dataset() const
{
    return m_dataset;
}
