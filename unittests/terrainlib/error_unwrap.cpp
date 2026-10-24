#include <catch2/catch_test_macros.hpp>

#include <string_view>

#include "error/Error.h"
#include "error/Result.h"
#include "error/unwrap.h"
#include "error/utils.h"

namespace {

enum class FileError {
    NotFound,
};

std::string_view format_as(const FileError) {
    return "file not found";
}

} // namespace

TEST_CASE("throwing_unwrap yields the value of a result that holds one", "[error][unwrap]") {
    const alp::Result<int, FileError> result = 7;

    CHECK(alp::throwing_unwrap(result) == 7);
}

TEST_CASE("throwing_unwrap returns for a void result that succeeded", "[error][unwrap]") {
    const alp::Result<void, FileError> result;

    CHECK_NOTHROW(alp::throwing_unwrap(result));
}

TEST_CASE("throwing_unwrap throws a panic for a result that holds an error", "[error][unwrap]") {
    const alp::Result<int, FileError> result = std::unexpected(alp::make_error(FileError::NotFound));

    CHECK_THROWS_AS(alp::throwing_unwrap(result), alp::Panic);
}

TEST_CASE("throwing_expect yields the value of a result that holds one", "[error][unwrap]") {
    const alp::Result<int, FileError> result = 7;

    CHECK(alp::throwing_expect(result, "reading the answer") == 7);
}

TEST_CASE("throwing_expect throws a panic for a result that holds an error", "[error][unwrap]") {
    const alp::Result<int, FileError> result = std::unexpected(alp::make_error(FileError::NotFound));

    CHECK_THROWS_AS(alp::throwing_expect(result, "reading the answer"), alp::Panic);
}
