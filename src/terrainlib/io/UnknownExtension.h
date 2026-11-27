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

#include <source_location>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include "common_error_codes.h"
#include "error/utils.h"
#include "range_utils.h"

namespace io {

// An extension that names no format the operation supports.
struct UnknownExtension {
    std::string extension;
    std::vector<std::string> supported;

    bool operator==(const UnknownExtension &) const = default;
};

inline std::string format_as(const UnknownExtension &error) {
    if (error.extension.empty()) {
        return fmt::format("missing extension, expected one of {}", fmt::join(error.supported, ", "));
    } else {
        return fmt::format("unsupported extension {}, expected one of {}", error.extension, fmt::join(error.supported, ", "));
    }
}

// Creates the error for an `extension` that is not one of `supported`.
[[nodiscard]]
inline alp::Error<Unsupported> make_unsupported_extension_error(
    const std::string_view extension,
    const std::span<const std::string_view> supported,
    const std::source_location location = std::source_location::current()) {
    UnknownExtension source{.extension = std::string(extension), .supported = to_vector<std::string>(supported)};
    return alp::make_error(Unsupported{}, std::move(source), location);
}

} // namespace io
