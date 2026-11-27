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
#include <string_view>

#include <fmt/format.h>

// A source location that formats as its function, without return type and parameters, followed by its file, line and column.
struct PrunedLocation {
    std::source_location location;
};

template <>
struct fmt::formatter<PrunedLocation> : fmt::formatter<std::string_view> {
    fmt::format_context::iterator format(PrunedLocation location, fmt::format_context &context) const;
};
