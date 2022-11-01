/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 madam
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

#include "Tiler.h"

#include <utility>

Tiler::Tiler(ctb::Grid  grid, const ctb::CRSBounds& bounds, Tile::Border border, Tile::Scheme scheme)
    : m_grid(std::move(grid))
    , m_bounds(bounds)
    , m_border_south_east(border)
    , m_scheme(scheme)
{

}

const ctb::Grid& Tiler::grid() const
{
    return m_grid;
}

ctb::i_tile Tiler::grid_size() const
{
    return grid().tileSize();
}

ctb::i_tile Tiler::tile_size() const
{
    return grid_size() + unsigned(border_south_east());
}

Tile::Border Tiler::border_south_east() const
{
    return m_border_south_east;
}

Tile::Scheme Tiler::scheme() const
{
    return m_scheme;
}

const ctb::CRSBounds& Tiler::bounds() const
{
    return m_bounds;
}

void Tiler::setBounds(const ctb::CRSBounds& newBounds)
{
    m_bounds = newBounds;
}

