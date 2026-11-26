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

#include <cstdint>
#include <limits>
#include <optional>
#include <string_view>
#include <type_traits>

#include "io/json/Value.h"

namespace json = io::json;

TEST_CASE("Value converts from strings, numbers and booleans but not from pointers or characters", "[io][json][Value]") {
    STATIC_REQUIRE(std::is_convertible_v<const char (&)[4], json::Value>);
    STATIC_REQUIRE(std::is_convertible_v<std::string_view, json::Value>);
    STATIC_REQUIRE(std::is_convertible_v<int8_t, json::Value>);
    STATIC_REQUIRE(std::is_convertible_v<bool, json::Value>);

    STATIC_REQUIRE_FALSE(std::is_constructible_v<json::Value, const char *>);
    STATIC_REQUIRE_FALSE(std::is_constructible_v<json::Value, char>);
    STATIC_REQUIRE_FALSE(std::is_constructible_v<json::Value, int *>);
}

TEST_CASE("Value compares equal integers of different types equal", "[io][json][Value]") {
    CHECK(json::Value(int64_t{1}) == json::Value(uint64_t{1}));
}

TEST_CASE("kind is Number for every integer and floating-point type", "[io][json][Value]") {
    CHECK(json::Value().kind() == json::Kind::Null);
    CHECK(json::Value(true).kind() == json::Kind::Boolean);
    CHECK(json::Value(int64_t{-1}).kind() == json::Kind::Number);
    CHECK(json::Value(uint64_t{1}).kind() == json::Kind::Number);
    CHECK(json::Value(0.5).kind() == json::Kind::Number);
    CHECK(json::Value("x").kind() == json::Kind::String);
    CHECK(json::Value(json::Array{}).kind() == json::Kind::Array);
    CHECK(json::Value(json::Object{}).kind() == json::Kind::Object);
}

TEST_CASE("as_int64 and as_uint64 return integers only within their range", "[io][json][Value]") {
    SECTION("negative integer as uint64_t") {
        CHECK(json::Value(int64_t{-1}).as_uint64() == std::nullopt);
    }
    SECTION("largest int64_t as int64_t") {
        CHECK(json::Value(uint64_t{std::numeric_limits<int64_t>::max()}).as_int64() == std::numeric_limits<int64_t>::max());
    }
    SECTION("beyond int64_t as int64_t") {
        CHECK(json::Value(uint64_t{std::numeric_limits<int64_t>::max()} + 1).as_int64() == std::nullopt);
    }
    SECTION("whole double as int64_t") {
        CHECK(json::Value(1.0).as_int64() == std::nullopt);
    }
}

TEST_CASE("as_double converts an integer", "[io][json][Value]") {
    CHECK(json::Value(3).as_double() == 3.0);
}

TEST_CASE("Object::insert_or_assign appends a new key and replaces an existing one in place", "[io][json][Value]") {
    SECTION("new key") {
        json::Object object{{"a", 1}, {"b", 2}};

        CHECK(object.insert_or_assign("c", 3));
        CHECK(object == json::Object{{"a", 1}, {"b", 2}, {"c", 3}});
    }
    SECTION("existing key") {
        json::Object object{{"a", 1}, {"b", 2}};

        CHECK_FALSE(object.insert_or_assign("a", 3));
        CHECK(object == json::Object{{"a", 3}, {"b", 2}});
    }
    SECTION("repeated key in the initializer list") {
        const json::Object object{{"a", 1}, {"b", 2}, {"a", 3}};

        CHECK(object == json::Object{{"a", 3}, {"b", 2}});
    }
}

TEST_CASE("Object::find matches whole keys only", "[io][json][Value]") {
    const json::Object object{{"ab", 1}, {"a", 2}};

    SECTION("present key") {
        const json::Object::const_iterator entry = object.find("a");

        REQUIRE(entry != object.end());
        CHECK(entry->second == json::Value(2));
        CHECK(object.contains("a"));
    }
    SECTION("missing key") {
        CHECK(object.find("b") == object.end());
        CHECK_FALSE(object.contains("b"));
    }
}
