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

#ifndef LAYERJSONWRITER_H
#define LAYERJSONWRITER_H

#include <string>

#include "ParallelTiler.h"

class MetaDataGenerator;
namespace ctb {
class Grid;
}

namespace layer_json_writer {
[[nodiscard]] std::string process(const MetaDataGenerator&, unsigned max_zoom = unsigned(-1));

namespace internal {
    [[nodiscard]] std::string tilejson(const std::string& s);
    [[nodiscard]] std::string name(const std::string& s);
    [[nodiscard]] std::string description(const std::string& s);
    [[nodiscard]] std::string version(const std::string& s);
    [[nodiscard]] std::string format();

    [[nodiscard]] std::string attribution(const std::string& s);
    [[nodiscard]] std::string schema(const Tile::Scheme& version);
    [[nodiscard]] std::string tiles();
    [[nodiscard]] std::string projection(int epsg_number);
    [[nodiscard]] std::string bounds(const ctb::Grid& grid);

    [[nodiscard]] std::string zoom_layer(const ctb::TileBounds& bounds);

}
}

#endif // LAYERJSONWRITER_H
