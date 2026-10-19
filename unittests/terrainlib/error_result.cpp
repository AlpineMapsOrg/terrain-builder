#include <catch2/catch_test_macros.hpp>

#include <expected>
#include <string_view>
#include <type_traits>

#include "error/Error.h"
#include "error/Result.h"
#include "error/utils.h"

namespace {

enum class FileError {
    NotFound,
};

std::string_view format_as(const FileError) {
    return "file not found";
}

enum class ParseError {
    BadDigit,
};

std::string_view format_as(const ParseError) {
    return "bad digit";
}

} // namespace

TEST_CASE("A result holds the error of its codes, with a nested set expanded", "[error][Result]") {
    STATIC_REQUIRE(std::is_same_v<alp::Result<int, FileError>, std::expected<int, alp::BasicError<FileError>>>);
    STATIC_REQUIRE(std::is_same_v<alp::Result<int, alp::Error<FileError>, ParseError>, std::expected<int, alp::BasicError<FileError, ParseError>>>);
}

TEST_CASE("A result converts from a result whose error converts, but not from a plain code", "[error][Result]") {
    STATIC_REQUIRE(std::is_convertible_v<alp::Result<int, FileError>, alp::Result<int>>);
    STATIC_REQUIRE(std::is_convertible_v<alp::Result<int, FileError>, alp::Result<int, FileError, ParseError>>);
    STATIC_REQUIRE(!std::is_convertible_v<alp::Result<int>, alp::Result<int, FileError>>);
    STATIC_REQUIRE(!std::is_convertible_v<std::unexpected<FileError>, alp::Result<int, FileError>>);
    STATIC_REQUIRE(!std::is_convertible_v<std::expected<int, FileError>, alp::Result<int, FileError>>);
}

TEST_CASE("A typed result converts to an erased result", "[error][Result]") {
    const alp::Result<int, FileError> succeeded = 3;
    const alp::Result<int> erased_success = succeeded;
    CHECK(erased_success.value() == 3);

    const alp::Result<int, FileError> failed = alp::make_unexpected(FileError::NotFound, "opening app.cfg");
    const alp::Result<int> erased_failure = failed;
    REQUIRE(!erased_failure.has_value());
    CHECK(erased_failure.error().holds(FileError::NotFound));
    REQUIRE(erased_failure.error().frames().size() == 1);
    CHECK(erased_failure.error().frames()[0].message == "opening app.cfg");
}

TEST_CASE("An unexpected error converts into a result whose error includes its codes", "[error][Result]") {
    const alp::Result<int, FileError, ParseError> set_result = alp::make_unexpected(ParseError::BadDigit);
    REQUIRE(!set_result.has_value());
    CHECK(set_result.error().holds(ParseError::BadDigit));

    const alp::Result<int> erased_result = alp::make_unexpected(FileError::NotFound);
    REQUIRE(!erased_result.has_value());
    CHECK(erased_result.error().holds(FileError::NotFound));
}

TEST_CASE("A result without a value carries an error", "[error][Result]") {
    const alp::Result<void, FileError> result = alp::make_unexpected(FileError::NotFound);

    REQUIRE(!result.has_value());
    CHECK(result.error().code() == FileError::NotFound);
}
