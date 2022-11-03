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

#include "ctb/types.hpp"
#include <glm/glm.hpp>
#include <glm/vector_relational.hpp>
#include <tuple>

// rename to TileDescriptor or TileMetaInfo or something
struct Tile {
    enum class Border {
        Yes = 1,
        No = 0
    };

    // The difference between TMS and slippyMap is whether y starts counting from the bottom (south) or top (north).
    // https://www.maptiler.com/google-maps-coordinates-tile-bounds-projection/#1/-16.88/79.02
    //
    enum class Scheme {
        Tms, // southern most tile is y = 0
        SlippyMap // aka Google, XYZ, webmap tiles; northern most tile is y = 0
    };

    struct Id {
        unsigned zoom_level = unsigned(-1);
        glm::uvec2 coords;
        Scheme scheme = Scheme::Tms;

        [[nodiscard]] Id to(Scheme new_scheme) const
        {
            if (scheme == new_scheme)
                return *this;

            const auto n_y_tiles = (1u << zoom_level);
            const auto coord_y = n_y_tiles - coords.y - 1;
            return { zoom_level, { coords.x, coord_y }, new_scheme };
        }
        [[nodiscard]] Id parent() const { return { zoom_level - 1, coords / 2u, scheme }; }
        [[nodiscard]] std::array<Tile::Id, 4> children() const
        {
            return {
                Tile::Id { zoom_level + 1, coords * 2u + glm::uvec2(0, scheme != Tile::Scheme::Tms), scheme },
                Tile::Id { zoom_level + 1, coords * 2u + glm::uvec2(1, scheme != Tile::Scheme::Tms), scheme },
                Tile::Id { zoom_level + 1, coords * 2u + glm::uvec2(0, scheme == Tile::Scheme::Tms), scheme },
                Tile::Id { zoom_level + 1, coords * 2u + glm::uvec2(1, scheme == Tile::Scheme::Tms), scheme }
            };
        }
        bool operator==(const Id& other) const { return other.coords == coords && other.scheme == scheme && other.zoom_level == zoom_level; };
        bool operator<(const Id& other) const { return std::tie(zoom_level, coords.x, coords.y, scheme) < std::tie(other.zoom_level, other.coords.x, other.coords.y, other.scheme); };
    };

    // used to generate file name
    Id tile_id;

    // srsBounds are the bounds of the tile including the border pixel.
    ctb::CRSBounds srsBounds;
    int srs_epsg;

    // some tiling schemes require a border (e.g. cesium heightmap https://github.com/CesiumGS/cesium/wiki/heightmap-1%2E0).
    // grid bounds does not contain that border (e.g. 64 width)
    // tile bounds contains that border (e.g. 65 width)
    ctb::i_tile gridSize;
    ctb::i_tile tileSize;
};
