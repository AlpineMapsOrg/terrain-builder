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

#include "error/Trace.h"

// Whether `a` and `b` name the same file, function, line and column.
inline bool is_same_location(const std::source_location &a, const std::source_location &b) {
    return std::string_view(a.file_name()) == b.file_name()
        && std::string_view(a.function_name()) == b.function_name()
        && a.line() == b.line()
        && a.column() == b.column();
}

// Whether `a` and `b` hold the same message at the same location.
inline bool is_same_frame(const alp::Frame &a, const alp::Frame &b) {
    return a.message == b.message && is_same_location(a.location, b.location);
}
