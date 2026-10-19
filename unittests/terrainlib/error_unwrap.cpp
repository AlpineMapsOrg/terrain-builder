#include <catch2/catch_test_macros.hpp>

#include <source_location>
#include <string>
#include <string_view>

#include <fmt/format.h>

#include "error/Error.h"
#include "error/Result.h"
#include "error/format.h"
#include "error/unwrap.h"
#include "error/utils.h"

namespace {

enum class FileError {
    NotFound,
};

std::string_view format_as(const FileError) {
    return "file not found";
}

// The message of the Panic that `body` throws.
template <typename F>
std::string catch_panic(F &&body) {
    try {
        body();
    } catch (const alp::Panic &panic) {
        return panic.what();
    }
    FAIL("no panic was thrown");
    return {};
}

} // namespace

TEST_CASE("unwrap yields the value of a result that holds one", "[error][unwrap]") {
    const alp::Result<int, FileError> result = 7;

    CHECK(alp::unwrap(result) == 7);
}

TEST_CASE("unwrap returns for a void result that succeeded", "[error][unwrap]") {
    const alp::Result<void, FileError> result;

    CHECK_NOTHROW(alp::unwrap(result));
}

TEST_CASE("unwrap throws a panic holding the report of the error, with a frame where it was unwrapped", "[error][unwrap]") {
    const alp::Error<FileError> error = alp::make_error(FileError::NotFound);
    const alp::Result<int, FileError> result = std::unexpected(error);
    const std::source_location location = std::source_location::current();

    const std::string message = catch_panic([&] { alp::unwrap(result, location); });

    CHECK(message == fmt::format("{:?}", alp::Error<FileError>(error).propagate(location)));
}

TEST_CASE("expect yields the value of a result that holds one", "[error][unwrap]") {
    const alp::Result<int, FileError> result = 7;

    CHECK(alp::expect(result, "reading the answer") == 7);
}

TEST_CASE("expect throws a panic holding the report of the error, with its context as the outermost frame", "[error][unwrap]") {
    const alp::Error<FileError> error = alp::make_error(FileError::NotFound);
    const alp::Result<int, FileError> result = std::unexpected(error);
    const std::source_location location = std::source_location::current();

    const std::string message = catch_panic([&] { alp::expect(result, "reading the answer", location); });

    CHECK(message == fmt::format("{:?}", alp::Error<FileError>(error).context("reading the answer", location)));
}
