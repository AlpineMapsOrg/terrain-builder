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
#include <filesystem>
#include <optional>
#include <string>
#include <utility>

#include "Error.h"
#include "io/envelope.h"
#include "octree/storage/IndexFile.h"
#include "store/inputs.h"

namespace sf_merger::inputs {

// Everything that determines the merged meshes; a cache must match it.
struct Record {
    // Optional members defeat zpp_bits' member counting.
    using serialize = zpp::bits::members<4>;

    store::inputs::Fingerprint base;
    store::inputs::Fingerprint added;
    std::optional<std::string> mask;
    // Bump on changes that alter the merged meshes.
    std::uint32_t processing_version = 1;

    bool operator==(const Record&) const = default;
};

using Schema = io::envelope::PayloadSchema<"sf_merger.Inputs", io::envelope::Version<1, Record>>;

inline Expected<Record> make_record(const std::filesystem::path& base, const std::filesystem::path& added, const std::optional<std::filesystem::path>& mask)
{
    Record record;
    for (auto [path, fingerprint] : { std::pair { &base, &record.base }, std::pair { &added, &record.added } }) {
        auto result = store::inputs::fingerprint(*path, octree::storage::metadata_file_name, octree::storage::index_file_name);
        if (!result) {
            return Error::propagate(std::move(result), "fingerprint SF merge input \"" + path->string() + "\"");
        }
        *fingerprint = std::move(*result);
    }
    if (mask.has_value()) {
        record.mask = std::filesystem::absolute(*mask).lexically_normal().string();
    }
    return record;
}

} // namespace sf_merger::inputs
