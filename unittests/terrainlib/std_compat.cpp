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

#include <type_traits>

#include "std_compat.h"

namespace {

struct Owner {};
struct Member {};

} // namespace

TEST_CASE("like_t takes the value category of T", "[std_compat]") {
    STATIC_REQUIRE(std::is_same_v<std_compat::like_t<Owner &, Member>, Member &>);
    STATIC_REQUIRE(std::is_same_v<std_compat::like_t<Owner, Member>, Member &&>);
    STATIC_REQUIRE(std::is_same_v<std_compat::like_t<Owner &&, Member>, Member &&>);
}

TEST_CASE("like_t adds the constness of T", "[std_compat]") {
    STATIC_REQUIRE(std::is_same_v<std_compat::like_t<const Owner &, Member>, const Member &>);
    STATIC_REQUIRE(std::is_same_v<std_compat::like_t<const Owner, Member>, const Member &&>);
    STATIC_REQUIRE(std::is_same_v<std_compat::like_t<const Owner &&, Member>, const Member &&>);
}

TEST_CASE("like_t keeps the constness of U", "[std_compat]") {
    STATIC_REQUIRE(std::is_same_v<std_compat::like_t<Owner &, const Member>, const Member &>);
    STATIC_REQUIRE(std::is_same_v<std_compat::like_t<Owner, const Member>, const Member &&>);
}

TEST_CASE("like_t ignores the reference of U", "[std_compat]") {
    STATIC_REQUIRE(std::is_same_v<std_compat::like_t<Owner, Member &>, Member &&>);
    STATIC_REQUIRE(std::is_same_v<std_compat::like_t<Owner &, Member &&>, Member &>);
}

TEST_CASE("forward_like returns its argument as like_t", "[std_compat]") {
    Member member;
    STATIC_REQUIRE(std::is_same_v<decltype(std_compat::forward_like<Owner &>(member)), Member &>);
    STATIC_REQUIRE(std::is_same_v<decltype(std_compat::forward_like<const Owner>(member)), const Member &&>);

    const Member &forwarded = std_compat::forward_like<Owner>(member);
    CHECK(&forwarded == &member);
}
