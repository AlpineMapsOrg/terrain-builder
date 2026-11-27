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

#include <cstdint>

#include <catch2/catch_test_macros.hpp>

#include "raster_store/StoreTraits.h"

TEST_CASE("raster store traits validate tile boundaries", "[raster_store][StoreTraits]")
{
    using Keys = raster_store::KeyTraits;
    using Tree = raster_store::HierarchyTraits;
    using Key = Keys::Key;

    CHECK(Keys::is_valid(Key { 0, { 0, 0 } }));
    CHECK_FALSE(Keys::is_valid(Key { 0, { 1, 0 } }));
    CHECK(Keys::is_valid(Key { 31, { uint32_t { 0x7fffffff }, uint32_t { 0x7fffffff } } }));
    CHECK_FALSE(Keys::is_valid(Key { 31, { uint32_t { 0x80000000 }, 0 } }));
    CHECK(Keys::is_valid(Key { 32, { 0, 0 } }));
    CHECK(Keys::is_valid(Key { 32, { UINT32_MAX, UINT32_MAX } }));
    CHECK_FALSE(Keys::is_valid(Key { 33, { 0, 0 } }));

    CHECK_FALSE(Tree::parent(Tree::root()).has_value());
    CHECK_FALSE(Tree::children(Key { 32, { UINT32_MAX, UINT32_MAX } }).has_value());

    const Key level_31 { 31, { uint32_t { 0x7fffffff }, uint32_t { 0x7fffffff } } };
    const auto children = Tree::children(level_31);
    REQUIRE(children.has_value());
    CHECK(children->at(0) == Key { 32, { UINT32_MAX - 1, UINT32_MAX - 1 } });
    CHECK(children->at(1) == Key { 32, { UINT32_MAX, UINT32_MAX - 1 } });
    CHECK(children->at(2) == Key { 32, { UINT32_MAX - 1, UINT32_MAX } });
    CHECK(children->at(3) == Key { 32, { UINT32_MAX, UINT32_MAX } });
}
