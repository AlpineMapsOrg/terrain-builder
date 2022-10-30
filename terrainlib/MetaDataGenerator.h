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

#pragma once

#include <string>
#include <vector>

#include "Dataset.h"
#include "ctb/Grid.hpp"
#include "ParallelTiler.h"
#include "ctb/types.hpp"


class MetaDataGenerator
{
public:
    MetaDataGenerator(const DatasetPtr& dataset, const ctb::Grid& grid, const ParallelTiler& tiler);
    [[nodiscard]] static MetaDataGenerator make(const std::string& input_data_path, ctb::Grid::Srs srs, Tile::Scheme tiling_scheme);
    [[nodiscard]] const ctb::Grid& grid() const;
    [[nodiscard]] const ParallelTiler& tiler() const;

    [[nodiscard]] std::vector<ctb::TileBounds> availableTiles(unsigned max_zoom = unsigned(-1)) const;
    [[nodiscard]] const DatasetPtr& dataset() const;

private:
    DatasetPtr m_dataset;
    ctb::Grid m_grid;
    ParallelTiler m_tiler;
};
