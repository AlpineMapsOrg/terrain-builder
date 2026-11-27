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
#include <type_traits>
#include <utility>

template <typename T>
constexpr size_t sizeof_val(const T &value) {
    if constexpr (requires { value.size_bytes(); }) {
        return value.size_bytes();
    } else {
        return sizeof(value);
    }
}

template <typename... T>
constexpr size_t sizeof_val(const T &...values) {
    return (sizeof_val(values) + ... + size_t{0});
}

// Destroys `value`, which must be moved in.
template <typename T>
    requires(!std::is_lvalue_reference_v<T>)
void drop(T &&value) {
    [[maybe_unused]] const T dropped(std::move(value));
}
