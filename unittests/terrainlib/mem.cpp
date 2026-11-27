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

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <utility>

#include "containers/UniqueResource.h"
#include "mem.h"

namespace {

template <typename T>
concept Droppable = requires(T &&value) { drop(std::forward<T>(value)); };

} // namespace

TEST_CASE("drop destroys the value moved into it", "[mem]") {
    bool released = false;
    UniqueResource dropped(&released, [](bool *flag) noexcept { *flag = true; });

    drop(std::move(dropped));

    CHECK(released);
}

TEST_CASE("drop only accepts a value that is moved in", "[mem]") {
    STATIC_REQUIRE(Droppable<std::string>);
    STATIC_REQUIRE(Droppable<std::string &&>);
    STATIC_REQUIRE_FALSE(Droppable<std::string &>);
    STATIC_REQUIRE_FALSE(Droppable<const std::string &>);
}
