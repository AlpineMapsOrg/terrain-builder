/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include <cstdint>
#include <optional>
#include <string>

#include "io/envelope.h"

namespace terrainbuilder::inputs {

// Everything that determines the meshes of a batch build; a cache must match it.
struct Record {
    // Optional members defeat zpp_bits' member counting.
    using serialize = zpp::bits::members<8>;

    std::string dataset;
    std::optional<std::string> textures;
    std::optional<std::uint32_t> min_texture_level;
    std::optional<std::uint32_t> max_texture_level;
    std::string mesh_srs;
    std::uint32_t target_level = 0;
    std::string format;
    // Bump on changes that alter the produced meshes.
    std::uint32_t processing_version = 1;

    bool operator==(const Record&) const = default;
};

using Schema = io::envelope::PayloadSchema<"sf_builder.Inputs", io::envelope::Version<1, Record>>;

} // namespace terrainbuilder::inputs
