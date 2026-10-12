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

#include <filesystem>
#include <string>
#include <string_view>

#include <expected>

#include "Error.h"
#include "io/byte.h"
#include "io/envelope.h"
#include "store/snapshot.h"

// An incomplete snapshot records the inputs and settings that produced it, so that a
// later run can verify that the snapshot is a compatible cache before reusing its nodes.
namespace store::inputs {

inline constexpr std::string_view file_name = "inputs.tmp";

// Identifies a published input snapshot by the recorded payload hashes of its metadata and index.
struct Fingerprint {
    std::string path;
    ::io::byte::vector metadata_hash;
    ::io::byte::vector index_hash;

    bool operator==(const Fingerprint&) const = default;
};

Expected<Fingerprint> fingerprint(const std::filesystem::path& snapshot, std::string_view metadata_file_name, std::string_view index_file_name);

template <typename Schema>
Expected<void> write(const std::filesystem::path& snapshot, const typename Schema::latest_type& record)
{
    auto written = ::io::envelope::write_to_path<Schema>(record, snapshot / file_name);
    if (!written) {
        return Error::propagate(std::move(written), "write input record");
    }
    return {};
}

template <typename Schema>
Expected<void> validate_cache(const std::filesystem::path& cache, const typename Schema::latest_type& record)
{
    if (!snapshot::is_partial(cache)) {
        return Error::fail(Error::Code::InvalidInput, "a cache must be an incomplete .part snapshot", cache);
    }
    auto saved = ::io::envelope::read_from_path<Schema>(cache / file_name);
    if (!saved) {
        return Error::propagate(std::move(saved), "requested cache has a missing or corrupt " + std::string(file_name));
    }
    if (*saved != record) {
        return Error::fail(Error::Code::InvalidInput, "requested cache was produced from different inputs or settings", cache);
    }
    return {};
}

} // namespace store::inputs
