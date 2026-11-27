/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include <fmt/format.h>
#include <libassert/assert.hpp>

namespace store {

// What a storage needs of its keys: formatting, hashing and a validity check.
template <typename Traits>
concept KeyTraits = fmt::formattable<typename Traits::Key> && requires(typename Traits::Key key) {
    typename Traits::Hasher;
    { Traits::is_valid(key) } -> std::same_as<bool>;
};

// Asserts that `key` is valid, which every function taking a key requires.
template <KeyTraits Traits>
void assert_valid(const typename Traits::Key& key) {
    ASSERT(Traits::is_valid(key), fmt::format("{}", key));
}

} // namespace store
