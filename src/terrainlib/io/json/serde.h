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

#include <concepts>
#include <ranges>
#include <string>
#include <string_view>

#include "common_error_codes.h"
#include "error/Result.h"
#include "io/json/Value.h"

namespace io::json {

// A map whose keys are strings, which converts to a JSON object.
template <typename T>
concept ObjectLike = std::ranges::input_range<const T> && String<typename T::key_type> && requires { typename T::mapped_type; };

// A range other than a string or a map, which converts to a JSON array.
template <typename T>
concept ArrayLike = std::ranges::input_range<const T> && !String<T> && !ObjectLike<T>;

// The layout of serialized JSON.
enum class Format {
    // No whitespace between tokens.
    Compact,
    // One entry per line, indented by nesting depth.
    Pretty,
};

// Converts `value` to JSON, recursing into the elements of maps and ranges.
template <typename T>
Value to_value(const T &value) {
    if constexpr (std::same_as<T, Value> || std::same_as<T, Array> || std::same_as<T, Object>) {
        return Value(value);
    } else if constexpr (std::same_as<T, std::nullptr_t>) {
        return Value(nullptr);
    } else if constexpr (Boolean<T> || Integer<T> || FloatingPoint<T> || String<T>) {
        return Value(value);
    } else if constexpr (ObjectLike<T>) {
        Object result;
        for (const auto &[key, item] : value) {
            const std::string_view key_view = key;
            result.insert_or_assign(std::string(key_view), to_value(item));
        }
        return result;
    } else if constexpr (ArrayLike<T>) {
        Array result;
        for (const auto &item : value) {
            result.push_back(to_value(item));
        }
        return result;
    } else {
        static_assert(false, "Unsupported JSON type");
    }
}

// Serializes `value`.
// Fails with Unsupported, giving the JSON pointer to the value as context, for a non-finite number or a string or key containing a NUL character.
alp::Result<std::string, Unsupported> to_string(const Value &value, Format format = Format::Compact);

} // namespace io::json
