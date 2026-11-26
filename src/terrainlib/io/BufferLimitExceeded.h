/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Martin Braunsperger
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

#include <cstddef>
#include <optional>
#include <string>

#include <fmt/format.h>

namespace io {

// A buffer would be larger than its size limit.
struct BufferLimitExceeded {
    explicit BufferLimitExceeded(const size_t limit) : limit(limit) {}
    explicit BufferLimitExceeded(const size_t limit, const size_t size) : limit(limit), size(size) {}

    // The size limit that applied, in bytes.
    size_t limit;
    // The size the buffer would have had, in bytes, if it is known.
    std::optional<size_t> size;

    bool operator==(const BufferLimitExceeded &) const = default;
};

inline std::string format_as(const BufferLimitExceeded &error) {
    if (error.size) {
        return fmt::format("a buffer of {} bytes would be larger than its size limit of {} bytes", *error.size, error.limit);
    } else {
        return fmt::format("a buffer would be larger than its size limit of {} bytes", error.limit);
    }
}

} // namespace io
