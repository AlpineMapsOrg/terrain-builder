/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 Adam Celarek
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

#include "ctb/Grid.hpp"
#include "ctb/types.hpp"

#include "Tile.h"

class Tiler
{
public:
    Tiler(ctb::Grid  grid, const ctb::CRSBounds& bounds, Tile::Border border, Tile::Scheme scheme);

    [[nodiscard]] Tile::Scheme scheme() const;
    [[nodiscard]] const ctb::CRSBounds& bounds() const;
    void setBounds(const ctb::CRSBounds& newBounds);

protected:
    [[nodiscard]] const ctb::Grid& grid() const;
    [[nodiscard]] ctb::i_tile grid_size() const;
    [[nodiscard]] ctb::i_tile tile_size() const;
    [[nodiscard]] Tile::Border border_south_east() const;

private:

    const ctb::Grid m_grid;
    ctb::CRSBounds m_bounds;
    const Tile::Border m_border_south_east;
    const Tile::Scheme m_scheme;
};

