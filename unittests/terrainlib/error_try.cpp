#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <expected>
#include <memory>
#include <optional>
#include <source_location>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "../tracked.h"
#include "error/Error.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"

namespace {

enum class FileError {
    NotFound,
};

std::string_view format_as(const FileError) {
    return "file not found";
}

} // namespace

TEST_CASE("ALP_TRY binds the value of a succeeded result", "[error][try]") {
    const auto forward = [](alp::Result<int, FileError> source) -> alp::Result<int, FileError> {
        ALP_TRY(const int value, std::move(source));
        return value + 1;
    };

    CHECK(forward(3).value() == 4);
}

TEST_CASE("ALP_TRY passes a failed result on with a frame at its line", "[error][try]") {
    std::source_location before;
    const auto forward = [&before](alp::Result<int, FileError> source) -> alp::Result<int, FileError> {
        before = std::source_location::current();
        ALP_TRY(const int value, std::move(source));
        return value + 1;
    };
    const alp::Result<int, FileError> result = forward(alp::make_error(FileError::NotFound));

    REQUIRE(!result.has_value());
    CHECK(result.error().code() == FileError::NotFound);
    REQUIRE(result.error().frames().size() == 2);
    CHECK(result.error().frames()[1].message == std::nullopt);
    CHECK(result.error().frames()[1].location.line() == before.line() + 1);
}

TEST_CASE("ALP_TRY without a target passes a failed result on with a frame at its line", "[error][try]") {
    std::source_location before;
    const auto skip = [&before](alp::Result<void, FileError> source) -> alp::Result<void, FileError> {
        before = std::source_location::current();
        ALP_TRY(std::move(source));
        return {};
    };
    CHECK(skip({}).has_value());

    const alp::Result<void, FileError> result = skip(alp::make_error(FileError::NotFound));
    REQUIRE(!result.has_value());
    REQUIRE(result.error().frames().size() == 2);
    CHECK(result.error().frames()[1].location.line() == before.line() + 1);
}

TEST_CASE("ALP_TRY_CONTEXT adds one frame with its message at its line", "[error][try]") {
    std::source_location before;
    const auto forward = [&before](alp::Result<int, FileError> source) -> alp::Result<int, FileError> {
        before = std::source_location::current();
        ALP_TRY_CONTEXT(const int value, std::move(source), "loading the configuration");
        return value + 1;
    };
    const alp::Result<int, FileError> result = forward(alp::make_error(FileError::NotFound));

    REQUIRE(!result.has_value());
    REQUIRE(result.error().frames().size() == 2);
    CHECK(result.error().frames()[1].message == "loading the configuration");
    CHECK(result.error().frames()[1].location.line() == before.line() + 1);
}

TEST_CASE("ALP_TRY_CONTEXT builds its message only on failure", "[error][try]") {
    uint32_t messages = 0;
    const auto make_message = [&messages]() {
        messages++;
        return std::string("loading the configuration");
    };
    const auto forward = [&make_message](alp::Result<int, FileError> source) -> alp::Result<int, FileError> {
        ALP_TRY_CONTEXT(const int value, std::move(source), make_message());
        return value + 1;
    };
    const auto skip = [&make_message](alp::Result<void, FileError> source) -> alp::Result<void, FileError> {
        ALP_TRY_CONTEXT(std::move(source), make_message());
        return {};
    };

    CHECK(forward(3).value() == 4);
    CHECK(skip({}).has_value());
    CHECK(messages == 0);

    CHECK(!forward(alp::make_error(FileError::NotFound)).has_value());
    CHECK(!skip(alp::make_error(FileError::NotFound)).has_value());
    CHECK(messages == 2);
}

TEST_CASE("ALP_TRY_CONTEXT takes a lazy message and builds it only on failure", "[error][try]") {
    uint32_t messages = 0;
    const auto make_message = [&messages]() {
        messages++;
        return std::string("loading the configuration");
    };
    const auto forward = [&make_message](alp::Result<int, FileError> source) -> alp::Result<int, FileError> {
        ALP_TRY_CONTEXT(const int value, std::move(source), make_message);
        return value + 1;
    };

    CHECK(forward(3).value() == 4);
    CHECK(messages == 0);

    const alp::Result<int, FileError> result = forward(alp::make_error(FileError::NotFound));
    CHECK(messages == 1);
    REQUIRE(!result.has_value());
    REQUIRE(result.error().frames().size() == 2);
    CHECK(result.error().frames()[1].message == "loading the configuration");
}

TEST_CASE("ALP_TRY hands a plain code to an enclosing function whose error is that code", "[error][try]") {
    const auto forward = [](std::expected<int, FileError> source) -> std::expected<int, FileError> {
        ALP_TRY(const int value, std::move(source));
        return value + 1;
    };
    CHECK(forward(3).value() == 4);

    const std::expected<int, FileError> result = forward(std::unexpected(FileError::NotFound));
    REQUIRE(!result.has_value());
    CHECK(result.error() == FileError::NotFound);
}

TEST_CASE("ALP_TRY turns a plain code into an error that originates at its line for an enclosing function that returns an Error", "[error][try]") {
    std::source_location before;
    const auto forward = [&before](std::expected<int, FileError> source) -> alp::Result<int, FileError> {
        before = std::source_location::current();
        ALP_TRY(const int value, std::move(source));
        return value + 1;
    };
    CHECK(forward(3).value() == 4);

    const alp::Result<int, FileError> result = forward(std::unexpected(FileError::NotFound));
    REQUIRE(!result.has_value());
    CHECK(result.error().code() == FileError::NotFound);
    CHECK(result.error().frames().size() == 1);
    CHECK(result.error().origin().line() == before.line() + 1);
}

TEST_CASE("ALP_TRY_CONTEXT adds its message to a plain code as the first context", "[error][try]") {
    std::source_location before;
    const auto forward = [&before](std::expected<int, FileError> source) -> alp::Result<int, FileError> {
        before = std::source_location::current();
        ALP_TRY_CONTEXT(const int value, std::move(source), "reading app.cfg");
        return value + 1;
    };
    const alp::Result<int, FileError> result = forward(std::unexpected(FileError::NotFound));

    REQUIRE(!result.has_value());
    REQUIRE(result.error().frames().size() == 1);
    CHECK(result.error().frames()[0].message == "reading app.cfg");
    CHECK(result.error().origin().line() == before.line() + 1);
    CHECK(result.error().frames()[0].location.line() == before.line() + 1);
}

TEST_CASE("ALP_TRY passes any other error through unchanged", "[error][try]") {
    const auto forward = [](std::expected<int, std::string> source) -> std::expected<int, std::string> {
        ALP_TRY(const int value, std::move(source));
        return value + 1;
    };
    CHECK(forward(3).value() == 4);

    const std::expected<int, std::string> result = forward(std::unexpected(std::string("broken")));
    REQUIRE(!result.has_value());
    CHECK(result.error() == "broken");
}

TEST_CASE("ALP_TRY passes an empty optional on", "[error][try]") {
    const auto forward = [](std::optional<int> source) -> std::optional<int> {
        ALP_TRY(const int value, std::move(source));
        return value + 1;
    };
    const auto skip = [](std::optional<int> source) -> std::optional<int> {
        ALP_TRY(std::move(source));
        return 0;
    };

    CHECK(forward(3).value() == 4);
    CHECK(!forward(std::nullopt).has_value());
    CHECK(skip(3).has_value());
    CHECK(!skip(std::nullopt).has_value());
}

TEST_CASE("ALP_TRY assigns to an existing variable", "[error][try]") {
    const auto assign = [](alp::Result<int, FileError> source) -> alp::Result<int, FileError> {
        int existing = 0;
        ALP_TRY(existing, std::move(source));
        return existing;
    };

    CHECK(assign(3).value() == 3);
    CHECK(!assign(alp::make_error(FileError::NotFound)).has_value());
}

TEST_CASE("ALP_TRY assigns to a member and leaves it alone on failure", "[error][try]") {
    struct Holder {
        int value = 0;

        alp::Result<void, FileError> load(alp::Result<int, FileError> source) {
            ALP_TRY(this->value, std::move(source));
            return {};
        }
    };
    Holder holder;

    REQUIRE(holder.load(3).has_value());
    CHECK(holder.value == 3);

    CHECK(!holder.load(alp::make_error(FileError::NotFound)).has_value());
    CHECK(holder.value == 3);
}

TEST_CASE("ALP_TRY moves the value out instead of copying it", "[error][try]") {
    const auto forward = [](alp::Result<Tracked, FileError> source) -> alp::Result<Tracked, FileError> {
        ALP_TRY(Tracked value, std::move(source));
        return value;
    };
    const alp::Result<Tracked, FileError> result = forward(Tracked{});

    REQUIRE(result.has_value());
    CHECK(result->copies == 0);
    CHECK(result->moves > 0);
}

TEST_CASE("ALP_TRY binds a reference into the result", "[error][try]") {
    const auto read = [](alp::Result<std::unique_ptr<int>, FileError> source) -> alp::Result<int, FileError> {
        ALP_TRY(const std::unique_ptr<int> &pointer, std::move(source));
        return *pointer;
    };

    CHECK(read(std::make_unique<int>(5)).value() == 5);
}

TEST_CASE("ALP_TRY binds a move-only value", "[error][try]") {
    const auto forward = [](alp::Result<std::unique_ptr<int>, FileError> source) -> alp::Result<std::unique_ptr<int>, FileError> {
        ALP_TRY(std::unique_ptr<int> pointer, std::move(source));
        return pointer;
    };
    const alp::Result<std::unique_ptr<int>, FileError> result = forward(std::make_unique<int>(5));

    REQUIRE(result.has_value());
    CHECK(**result == 5);
}

TEST_CASE("ALP_TRY binds a structured binding with one name", "[error][try]") {
    struct Single {
        int value;
    };
    const auto unpack = [](alp::Result<Single, FileError> source) -> alp::Result<int, FileError> {
        ALP_TRY(const auto [value], std::move(source));
        return value;
    };

    CHECK(unpack(Single{.value = 4}).value() == 4);
}

TEST_CASE("ALP_TRY evaluates its expression once", "[error][try]") {
    uint32_t calls = 0;
    const auto load = [&calls](const alp::Result<int, FileError> &source) -> alp::Result<int, FileError> {
        calls++;
        return source;
    };
    const auto forward = [&load](const alp::Result<int, FileError> &source) -> alp::Result<int, FileError> {
        ALP_TRY(const int value, load(source));
        return value;
    };

    CHECK(forward(7).value() == 7);
    CHECK(calls == 1);

    CHECK(!forward(alp::make_error(FileError::NotFound)).has_value());
    CHECK(calls == 2);
}

TEST_CASE("ALP_TRY in a loop returns at the first failure and leaves the result it was given intact", "[error][try]") {
    const std::vector<alp::Result<int, FileError>> sources{
        alp::Result<int, FileError>(1),
        alp::Result<int, FileError>(alp::make_error(FileError::NotFound)),
        alp::Result<int, FileError>(3),
    };
    uint32_t visited = 0;
    const auto sum = [&sources, &visited]() -> alp::Result<int, FileError> {
        int total = 0;
        for (const alp::Result<int, FileError> &source : sources) {
            visited++;
            ALP_TRY(const int value, source);
            total += value;
        }
        return total;
    };
    const alp::Result<int, FileError> result = sum();

    REQUIRE(!result.has_value());
    CHECK(visited == 2);
    CHECK(result.error().frames().size() == 2);
    REQUIRE(!sources[1].has_value());
    CHECK(sources[1].error().frames().size() == 1);
}

TEST_CASE("Two ALP_TRY on one line do not collide", "[error][try]") {
    const auto add = [](alp::Result<int, FileError> first, alp::Result<int, FileError> second) -> alp::Result<int, FileError> {
        ALP_TRY(const int left, std::move(first)); ALP_TRY(const int right, std::move(second));
        return left + right;
    };

    CHECK(add(1, 2).value() == 3);
    CHECK(!add(1, alp::make_error(FileError::NotFound)).has_value());
}

TEST_CASE("ALP_TRY takes an expression with commas inside parentheses", "[error][try]") {
    const auto make_pair = [](const int first, const int second) -> alp::Result<std::pair<int, int>, FileError> {
        return std::pair<int, int>{first, second};
    };
    const auto add = [&make_pair]() -> alp::Result<int, FileError> {
        ALP_TRY(const auto pair, (make_pair(1, 2)));
        return pair.first + pair.second;
    };

    CHECK(add().value() == 3);
}

TEST_CASE("ALP_TRY_CONTEXT takes an expression with commas inside parentheses", "[error][try]") {
    const auto make_pair = [](const int first, const int second) -> alp::Result<std::pair<int, int>, FileError> {
        return std::pair<int, int>{first, second};
    };
    const auto add = [&make_pair]() -> alp::Result<int, FileError> {
        ALP_TRY_CONTEXT(const auto pair, (make_pair(1, 2)), "adding the pair");
        return pair.first + pair.second;
    }; 

    CHECK(add().value() == 3);
}

TEST_CASE("ALP_TRY_CONTEXT takes a message with commas and variables named like its parameters", "[error][try]") {
    const auto forward = [](alp::Result<int, FileError> result) -> alp::Result<int, FileError> {
        const std::string message = "app.cfg";
        ALP_TRY_CONTEXT(const int target, std::move(result), fmt::format("reading {}, then {}", message, "the index"));
        return target;
    };
    CHECK(forward(3).value() == 3);

    const alp::Result<int, FileError> result = forward(alp::make_error(FileError::NotFound));
    REQUIRE(!result.has_value());
    REQUIRE(result.error().frames().size() == 2);
    CHECK(result.error().frames()[1].message == "reading app.cfg, then the index");
}

TEST_CASE("Frames accumulate across several ALP_TRY", "[error][try]") {
    const auto fail_inner = []() -> alp::Result<int, FileError> {
        return alp::make_error(FileError::NotFound);
    };
    const auto fail_middle = [&fail_inner]() -> alp::Result<int, FileError> {
        ALP_TRY_CONTEXT(const int value, fail_inner(), "reading the configuration");
        return value;
    };
    const auto fail_outer = [&fail_middle]() -> alp::ErasedResult<int> {
        ALP_TRY(const int value, fail_middle());
        return value;
    };
    const alp::ErasedResult<int> result = fail_outer();

    REQUIRE(!result.has_value());
    REQUIRE(result.error().frames().size() == 3);
    CHECK(result.error().frames()[1].message == "reading the configuration");
    CHECK(result.error().frames()[2].message == std::nullopt);
}
