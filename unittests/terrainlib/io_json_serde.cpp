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
#include <catch2/generators/catch_generators.hpp>

#include <charconv>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <system_error>
#include <vector>

#include "../result_helpers.h"
#include "common_error_codes.h"
#include "error/prelude.h"
#include "io/json/serde.h"

using namespace std::literals;

namespace json = io::json;

TEST_CASE("to_value converts each kind of C++ type to its JSON kind", "[io][json][serde]") {
    SECTION("bool") {
        CHECK(json::to_value(true) == json::Value(true));
    }
    SECTION("string literal") {
        CHECK(json::to_value("ab") == json::Value("ab"));
    }
    SECTION("std::string") {
        CHECK(json::to_value("ab"s) == json::Value("ab"));
    }
    SECTION("nullptr") {
        CHECK(json::to_value(nullptr) == json::Value());
    }
    SECTION("map") {
        CHECK(json::to_value(std::map<std::string, int>{{"b", 2}, {"a", 1}}) == json::Object{{"a", 1}, {"b", 2}});
    }
    SECTION("nested vectors") {
        CHECK(json::to_value(std::vector<std::vector<int>>{{1, 2}, {3}}) == json::Array{json::Array{1, 2}, json::Array{3}});
    }
    SECTION("vector of bool") {
        CHECK(json::to_value(std::vector<bool>{true, false}) == json::Array{true, false});
    }
    SECTION("map of vectors") {
        CHECK(json::to_value(std::map<std::string, std::vector<int>>{{"a", {1, 2}}}) == json::Object{{"a", json::Array{1, 2}}});
    }
}

TEST_CASE("to_value passes JSON values through", "[io][json][serde]") {
    SECTION("vector of values") {
        CHECK(json::to_value(std::vector<json::Value>{1, "a"}) == json::Array{1, "a"});
    }
    SECTION("map of values") {
        CHECK(json::to_value(std::map<std::string, json::Value>{{"a", json::Array{1}}}) == json::Object{{"a", json::Array{1}}});
    }
    SECTION("object") {
        CHECK(json::to_value(json::Object{{"a", 1}}) == json::Object{{"a", 1}});
    }
}

TEST_CASE("to_string writes every kind compactly and keeps insertion order", "[io][json][serde]") {
    const json::Value value = json::Object{
        {"b", json::Array{nullptr, true, -1, 2u, 0.5, "x"}},
        {"a", json::Object{}},
    };

    const std::string text = REQUIRE_VALUE(json::to_string(value));

    CHECK(text == R"({"b":[null,true,-1,2,0.5,"x"],"a":{}})");
}

TEST_CASE("to_string writes a scalar at the top level", "[io][json][serde]") {
    SECTION("null") {
        CHECK(REQUIRE_VALUE(json::to_string(json::Value())) == "null");
    }
    SECTION("number") {
        CHECK(REQUIRE_VALUE(json::to_string(json::Value(1))) == "1");
    }
    SECTION("string") {
        CHECK(REQUIRE_VALUE(json::to_string(json::Value("x"))) == R"("x")");
    }
}

TEST_CASE("to_string writes 64-bit integers exactly", "[io][json][serde]") {
    const json::Value value = json::Array{std::numeric_limits<int64_t>::min(), std::numeric_limits<uint64_t>::max()};

    const std::string text = REQUIRE_VALUE(json::to_string(value));

    CHECK(text == "[-9223372036854775808,18446744073709551615]");
}

TEST_CASE("to_string escapes quotes, backslashes and control characters", "[io][json][serde]") {
    const std::string text = REQUIRE_VALUE(json::to_string(json::Value("a\"b\\c\nd")));

    CHECK(text == R"("a\"b\\c\nd")");
}

TEST_CASE("to_string writes a double that reads back unchanged", "[io][json][serde]") {
    const std::string text = REQUIRE_VALUE(json::to_string(json::Value(0.1)));

    double parsed = 0.0;
    const std::from_chars_result result = std::from_chars(text.data(), text.data() + text.size(), parsed);

    REQUIRE(result.ec == std::errc{});
    REQUIRE(result.ptr == text.data() + text.size());
    CHECK(parsed == 0.1);
}

TEST_CASE("to_string indents nested values in the pretty format", "[io][json][serde]") {
    const json::Value value = json::Object{{"a", json::Array{1, 2}}};

    const std::string text = REQUIRE_VALUE(json::to_string(value, json::Format::Pretty));

    CHECK(text == R"({
  "a":[
    1,
    2
  ]
})");
}

TEST_CASE("to_string rejects values JSON cannot represent", "[io][json][serde]") {
    SECTION("non-finite number") {
        const double number = GENERATE(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());
        const json::Value value = json::Array{json::Object{{"a", number}}};

        const auto error = REQUIRE_ERROR(json::to_string(value));

        CHECK(error.holds<Unsupported>());
    }
    SECTION("NUL in a string") {
        const json::Value value = json::Array{json::Object{{"a", "x\0y"s}}};

        const auto error = REQUIRE_ERROR(json::to_string(value));

        CHECK(error.holds<Unsupported>());
    }
    SECTION("NUL in a key") {
        const json::Value value = json::Array{json::Object{{"x\0y"s, 1}}};

        const auto error = REQUIRE_ERROR(json::to_string(value));

        CHECK(error.holds<Unsupported>());
    }
}
