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

#include "store/inputs.h"

#include <system_error>

namespace store::inputs {

namespace {
    Expected<::io::byte::vector> recorded_hash(const std::filesystem::path& path)
    {
        auto header = ::io::envelope::read_header(path);
        if (!header) {
            return Error::propagate(std::move(header), "fingerprint input");
        }
        // An empty hash would match any content.
        if (header->hash.empty()) {
            return Error::fail(Error::Code::Unsupported, "input file records no payload hash", path);
        }
        return std::move(header->hash);
    }
} // namespace

Expected<Fingerprint> fingerprint(const std::filesystem::path& snapshot, const std::string_view metadata_file_name, const std::string_view index_file_name)
{
    std::error_code error;
    const auto canonical = std::filesystem::canonical(snapshot, error);
    if (error) {
        return Error::fail(Error::Code::Io, "resolve input path", snapshot, error);
    }
    auto metadata = recorded_hash(canonical / metadata_file_name);
    if (!metadata) {
        return Error::propagate(std::move(metadata));
    }
    auto index = recorded_hash(canonical / index_file_name);
    if (!index) {
        return Error::propagate(std::move(index));
    }
    return Fingerprint { canonical.string(), std::move(*metadata), std::move(*index) };
}

} // namespace store::inputs
