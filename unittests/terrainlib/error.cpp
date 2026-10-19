#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <expected>
#include <memory>
#include <optional>
#include <source_location>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <fmt/format.h>

#include "../tracked.h"
#include "error/Error.h"
#include "error/Infallible.h"
#include "error/format.h"
#include "error/utils.h"
#include "error_test_utils.h"
#include "variant_utils.h"

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

// A code that carries data.
struct OffsetError {
    uint32_t offset;

    bool operator==(const OffsetError &) const = default;
};

std::string format_as(const OffsetError error) {
    return fmt::format("bad byte at {}", error.offset);
}

// A formattable class without equality.
struct Incomparable {};

std::string_view format_as(const Incomparable) {
    return "incomparable";
}

// A code that can only be moved.
struct MoveOnlyError {
    std::unique_ptr<int> value;
};

std::string_view format_as(const MoveOnlyError &) {
    return "move only";
}

// A code that counts how often it was copied and moved.
struct TrackedError {
    Tracked tracked;
};

std::string_view format_as(const TrackedError &) {
    return "tracked";
}

// Whether a context can be added to an lvalue of E, which would leave the original behind.
template <typename E>
concept accepts_context_on_lvalue = requires(E &error) { error.context(std::string(), std::source_location()); };

// Whether the codes Fs can be split off an E.
template <typename E, typename... Fs>
concept can_split_off = requires(E error) { std::move(error).template split<Fs...>(); };

} // namespace

TEST_CASE("An error without codes is erased", "[error]") {
    STATIC_REQUIRE(!alp::Error<FileError>::is_erased);
    STATIC_REQUIRE(alp::Error<>::is_erased);
}

TEST_CASE("An error is created explicitly from one of its codes, an erased error from a message", "[error]") {
    STATIC_REQUIRE(std::is_constructible_v<alp::Error<FileError>, FileError>);
    STATIC_REQUIRE(std::is_constructible_v<alp::Error<FileError, ParseError>, ParseError>);
    STATIC_REQUIRE(!std::is_convertible_v<FileError, alp::Error<FileError>>);
    STATIC_REQUIRE(!std::is_constructible_v<alp::Error<FileError>, ParseError>);
    STATIC_REQUIRE(!std::is_constructible_v<alp::Error<FileError, ParseError>, OffsetError>);
    STATIC_REQUIRE(!std::is_constructible_v<alp::Error<FileError>, std::string>);
    STATIC_REQUIRE(std::is_constructible_v<alp::Error<>, std::string>);
    STATIC_REQUIRE(!std::is_convertible_v<std::string, alp::Error<>>);
    STATIC_REQUIRE(!std::is_constructible_v<alp::Error<>, FileError>);
}

TEST_CASE("An error converts into any error that includes its codes, and into an erased error", "[error]") {
    STATIC_REQUIRE(std::is_convertible_v<alp::Error<FileError>, alp::Error<FileError, ParseError>>);
    STATIC_REQUIRE(std::is_convertible_v<const alp::Error<FileError> &, alp::Error<FileError, ParseError>>);
    STATIC_REQUIRE(std::is_convertible_v<alp::Error<FileError, ParseError>, alp::Error<ParseError, FileError>>);
    STATIC_REQUIRE(std::is_convertible_v<alp::Error<FileError, ParseError>, alp::Error<FileError, ParseError, OffsetError>>);
    STATIC_REQUIRE(std::is_convertible_v<alp::Error<FileError>, alp::Error<>>);
    STATIC_REQUIRE(std::is_convertible_v<alp::Error<FileError, ParseError>, alp::Error<>>);
}

TEST_CASE("An error does not convert into an error that lacks one of its codes", "[error]") {
    STATIC_REQUIRE(!std::is_constructible_v<alp::Error<FileError>, alp::Error<ParseError>>);
    STATIC_REQUIRE(!std::is_constructible_v<alp::Error<FileError>, alp::Error<FileError, ParseError>>);
    STATIC_REQUIRE(!std::is_constructible_v<alp::Error<FileError, ParseError>, alp::Error<OffsetError>>);
    STATIC_REQUIRE(!std::is_constructible_v<alp::Error<FileError>, alp::Error<>>);
    STATIC_REQUIRE(!std::is_constructible_v<alp::Error<FileError, ParseError>, alp::Error<>>);
}

TEST_CASE("A nested error set expands into its codes, each kept once in the order it first appears", "[error]") {
    STATIC_REQUIRE(std::is_same_v<alp::Error<FileError, ParseError>, alp::BasicError<FileError, ParseError>>);
    STATIC_REQUIRE(std::is_same_v<alp::Error<alp::Error<FileError, ParseError>, OffsetError>, alp::BasicError<FileError, ParseError, OffsetError>>);
    STATIC_REQUIRE(std::is_same_v<alp::Error<FileError, alp::Error<ParseError, FileError>>, alp::BasicError<FileError, ParseError>>);
    STATIC_REQUIRE(std::is_same_v<alp::Error<alp::Error<FileError, ParseError>>, alp::BasicError<FileError, ParseError>>);
}

TEST_CASE("An error narrows only to a set of codes it can hold", "[error]") {
    STATIC_REQUIRE(alp::Error<FileError, ParseError, OffsetError>::narrows_to<alp::Error<FileError, ParseError>>);
    STATIC_REQUIRE(!alp::Error<FileError>::narrows_to<alp::Error<FileError, ParseError>>);
    STATIC_REQUIRE(!alp::Error<FileError>::narrows_to<>);
    STATIC_REQUIRE(alp::Error<>::narrows_to<FileError>);
    STATIC_REQUIRE(!alp::Error<>::narrows_to<MoveOnlyError>);
}

TEST_CASE("An error splits only into codes it holds and at least one other", "[error]") {
    STATIC_REQUIRE(can_split_off<alp::Error<FileError, ParseError>, FileError>);
    STATIC_REQUIRE(!can_split_off<alp::Error<FileError, ParseError>, FileError, ParseError>);
    STATIC_REQUIRE(!can_split_off<alp::Error<FileError, ParseError>, OffsetError>);
}

TEST_CASE("An error converts as if Infallible were not among its codes", "[error]") {
    STATIC_REQUIRE(std::is_convertible_v<alp::Error<alp::Infallible>, alp::Error<FileError>>);
    STATIC_REQUIRE(std::is_convertible_v<alp::Error<alp::Infallible>, alp::Error<FileError, ParseError>>);
    STATIC_REQUIRE(std::is_convertible_v<alp::Error<alp::Infallible>, alp::Error<>>);
    STATIC_REQUIRE(std::is_convertible_v<alp::Error<FileError, alp::Infallible>, alp::Error<FileError>>);
    STATIC_REQUIRE(!std::is_constructible_v<alp::Error<alp::Infallible>, alp::Error<FileError>>);
    STATIC_REQUIRE(!std::is_constructible_v<alp::Error<FileError>, alp::Error<ParseError, alp::Infallible>>);
}

TEST_CASE("An error holds its code and originates at the location it is handed", "[error]") {
    const std::source_location location = std::source_location::current();
    const alp::Error<FileError> error(FileError::NotFound, location);

    CHECK(error.code() == FileError::NotFound);
    CHECK(is_same_location(error.origin(), location));
}

TEST_CASE("An erased error holds a message and originates at the location it is handed", "[error]") {
    const std::source_location location = std::source_location::current();
    const alp::Error<> error("round 3 made no progress", location);

    CHECK(fmt::to_string(error).contains("round 3 made no progress"));
    CHECK(is_same_location(error.origin(), location));
}

TEST_CASE("Context and propagate consume the error they are called on", "[error]") {
    using FileFailure = alp::Error<FileError>;

    STATIC_REQUIRE(std::is_same_v<decltype(std::declval<FileFailure>().context(std::string(), std::source_location())), FileFailure>);
    STATIC_REQUIRE(std::is_same_v<std::invoke_result_t<decltype(&FileFailure::propagate), FileFailure, std::source_location>, FileFailure>);
    STATIC_REQUIRE(!accepts_context_on_lvalue<FileFailure>);
    STATIC_REQUIRE(!std::is_invocable_v<decltype(&FileFailure::propagate), FileFailure &, std::source_location>);
}

TEST_CASE("Context adds a frame at the location it is handed", "[error]") {
    const std::source_location location = std::source_location::current();
    const auto error = alp::make_error(FileError::NotFound).context("reading the configuration", location);

    REQUIRE(error.frames().size() == 2);
    CHECK(error.frames()[1].message == "reading the configuration");
    CHECK(is_same_location(error.frames()[1].location, location));
}

TEST_CASE("Propagate adds a frame without a message at the location it is handed", "[error]") {
    const std::source_location location = std::source_location::current();
    const auto error = alp::make_error(FileError::NotFound).propagate(location);

    REQUIRE(error.frames().size() == 2);
    CHECK(error.frames()[1].message == std::nullopt);
    CHECK(is_same_location(error.frames()[1].location, location));
}

TEST_CASE("Context adds its frames in the order it is called", "[error]") {
    const auto error = alp::make_error(FileError::NotFound)
                           .context("inner")
                           .context("outer");

    REQUIRE(error.frames().size() == 3);
    CHECK(error.frames()[1].message == "inner");
    CHECK(error.frames()[2].message == "outer");
}

TEST_CASE("An error widens into a set, keeping its frames", "[error]") {
    const alp::Error<FileError, ParseError> widened = alp::make_error(FileError::NotFound).context("opening app.cfg");

    CHECK(widened.holds(FileError::NotFound));
    REQUIRE(widened.frames().size() == 2);
    CHECK(widened.frames()[1].message == "opening app.cfg");
}

TEST_CASE("A const error widens into a set and stays as it was", "[error]") {
    const auto error = alp::make_error(FileError::NotFound).context("opening app.cfg");
    const alp::Error<FileError, ParseError> widened = error;

    CHECK(widened.holds(FileError::NotFound));
    CHECK(widened.frames().size() == 2);
    CHECK(error.code() == FileError::NotFound);
    CHECK(error.frames().size() == 2);
}

TEST_CASE("Erasure keeps the code the error holds and its frames", "[error]") {
    const alp::Error<> erased = alp::Error<FileError, ParseError>(ParseError::BadDigit).context("parsing the header");

    CHECK(erased.holds(ParseError::BadDigit));
    REQUIRE(erased.frames().size() == 2);
    CHECK(erased.frames()[1].message == "parsing the header");
}

TEST_CASE("Erasing an error moves its code instead of copying it", "[error]") {
    const alp::Error<> erased = alp::Error<TrackedError>(TrackedError{});

    const TrackedError *code = erased.get_if<TrackedError>();
    REQUIRE(code != nullptr);
    CHECK(code->tracked.copies == 0);
}

TEST_CASE("An error with a move-only code does not erase", "[error]") {
    STATIC_REQUIRE(!std::is_constructible_v<alp::Error<>, alp::Error<MoveOnlyError>>);
}

TEST_CASE("An error with a move-only code widens into a set", "[error]") {
    const alp::Error<MoveOnlyError, FileError> widened = alp::make_error(MoveOnlyError{.value = std::make_unique<int>(5)});

    const MoveOnlyError *code = widened.get_if<MoveOnlyError>();
    REQUIRE(code != nullptr);
    CHECK(*code->value == 5);
}

TEST_CASE("narrow returns the narrowed error or the error it was called on", "[error]") {
    STATIC_REQUIRE(std::is_same_v<decltype(std::declval<alp::Error<FileError, ParseError>>().narrow<FileError>()), std::expected<alp::Error<FileError>, alp::Error<FileError, ParseError>>>);
    STATIC_REQUIRE(std::is_same_v<decltype(std::declval<alp::Error<FileError, ParseError, OffsetError>>().narrow<alp::Error<FileError, ParseError>>()), std::expected<alp::Error<FileError, ParseError>, alp::Error<FileError, ParseError, OffsetError>>>);
    STATIC_REQUIRE(std::is_same_v<decltype(std::declval<alp::Error<>>().narrow<FileError>()), std::expected<alp::Error<FileError>, alp::Error<>>>);
}

TEST_CASE("narrow gives the error with only the code it holds, keeping its frames", "[error]") {
    const auto result = alp::Error<FileError, ParseError>(FileError::NotFound).context("opening app.cfg").narrow<FileError>();

    REQUIRE(result.has_value());
    CHECK(result->code() == FileError::NotFound);
    REQUIRE(result->frames().size() == 2);
    CHECK(result->frames()[1].message == "opening app.cfg");
}

TEST_CASE("narrow hands the error back when it holds another code", "[error]") {
    const auto result = alp::Error<FileError, ParseError>(ParseError::BadDigit).context("parsing the header").narrow<FileError>();

    REQUIRE(!result.has_value());
    CHECK(result.error().holds(ParseError::BadDigit));
    CHECK(result.error().frames().size() == 2);
}

TEST_CASE("narrow on a const error leaves it as it was", "[error]") {
    const alp::Error<FileError, ParseError> error = alp::Error<FileError, ParseError>(FileError::NotFound).context("opening app.cfg");
    const auto result = error.narrow<FileError>();

    REQUIRE(result.has_value());
    CHECK(result->frames().size() == 2);
    CHECK(error.holds(FileError::NotFound));
    CHECK(error.frames().size() == 2);
}

TEST_CASE("narrow to several codes keeps those it names", "[error]") {
    const auto matched = alp::Error<FileError, ParseError, OffsetError>(ParseError::BadDigit).narrow<FileError, ParseError>();
    REQUIRE(matched.has_value());
    CHECK(matched->holds(ParseError::BadDigit));

    const auto missed = alp::Error<FileError, ParseError, OffsetError>(OffsetError{.offset = 7}).narrow<FileError, ParseError>();
    REQUIRE(!missed.has_value());
    CHECK(missed.error().holds(OffsetError{.offset = 7}));
}

TEST_CASE("narrow to a set gives the error with only that set's codes, keeping its frames", "[error]") {
    const auto result = alp::Error<alp::Error<FileError, ParseError>, OffsetError>(ParseError::BadDigit)
                            .context("parsing the header")
                            .narrow<alp::Error<FileError, ParseError>>();

    REQUIRE(result.has_value());
    CHECK(result->holds(ParseError::BadDigit));
    CHECK(result->frames().size() == 2);
}

TEST_CASE("narrow to a set hands the error back when it holds a code outside the set", "[error]") {
    const auto result = alp::Error<alp::Error<FileError, ParseError>, OffsetError>(OffsetError{.offset = 7})
                            .context("reading the body")
                            .narrow<alp::Error<FileError, ParseError>>();

    REQUIRE(!result.has_value());
    CHECK(result.error().holds(OffsetError{.offset = 7}));
    CHECK(result.error().frames().size() == 2);
}

TEST_CASE("narrow on an erased error gives the error with the code it was erased from, keeping its frames", "[error]") {
    const auto result = alp::Error<>(alp::make_error(FileError::NotFound).context("opening app.cfg")).narrow<FileError>();

    REQUIRE(result.has_value());
    CHECK(result->code() == FileError::NotFound);
    REQUIRE(result->frames().size() == 2);
    CHECK(result->frames()[1].message == "opening app.cfg");
}

TEST_CASE("narrow on an erased error hands it back when it holds another code", "[error]") {
    const auto result = alp::Error<>(alp::make_error(ParseError::BadDigit).context("parsing the header")).narrow<FileError>();

    REQUIRE(!result.has_value());
    CHECK(result.error().holds(ParseError::BadDigit));
    CHECK(result.error().frames().size() == 2);
}

TEST_CASE("An error with a move-only code narrows to its code", "[error]") {
    const auto result = alp::Error<MoveOnlyError, FileError>(MoveOnlyError{.value = std::make_unique<int>(5)}).narrow<MoveOnlyError>();

    REQUIRE(result.has_value());
    CHECK(*result->code().value == 5);
}

TEST_CASE("split returns the error with the named codes or the error with the others", "[error]") {
    using Source = alp::Error<FileError, ParseError, OffsetError>;
    using Result = decltype(std::declval<Source>().split<ParseError>());

    STATIC_REQUIRE(std::is_same_v<Result::value_type, alp::Error<ParseError>>);
    STATIC_REQUIRE(std::is_same_v<Result::error_type, alp::Error<FileError, OffsetError>>);
}

TEST_CASE("split gives the error with only the named code it holds", "[error]") {
    const auto result = alp::Error<FileError, ParseError>(FileError::NotFound).split<FileError>();

    REQUIRE(result.has_value());
    CHECK(result->code() == FileError::NotFound);
}

TEST_CASE("split gives the error with only its other codes when it holds one of those", "[error]") {
    const auto result = alp::Error<FileError, ParseError, OffsetError>(OffsetError{.offset = 7}).split<FileError>();

    REQUIRE(!result.has_value());
    CHECK(result.error().holds(OffsetError{.offset = 7}));
}

TEST_CASE("split on a const error leaves it as it was", "[error]") {
    const alp::Error<FileError, ParseError> error(ParseError::BadDigit);
    const auto result = error.split<FileError>();

    REQUIRE(!result.has_value());
    CHECK(error.holds(ParseError::BadDigit));
}

TEST_CASE("split off a set returns that set or the error with the others", "[error]") {
    using Source = alp::Error<FileError, ParseError, OffsetError>;
    using Set = alp::Error<FileError, ParseError>;
    using Result = decltype(std::declval<Source>().split<Set>());

    STATIC_REQUIRE(std::is_same_v<Result::value_type, Set>);
    STATIC_REQUIRE(std::is_same_v<Result::error_type, alp::Error<OffsetError>>);
}

TEST_CASE("An error with a move-only code keeps it when another code is split off", "[error]") {
    const auto result = alp::Error<MoveOnlyError, FileError>(MoveOnlyError{.value = std::make_unique<int>(5)}).split<FileError>();

    REQUIRE(!result.has_value());
    const MoveOnlyError *code = result.error().get_if<MoveOnlyError>();
    REQUIRE(code != nullptr);
    CHECK(*code->value == 5);
}

TEST_CASE("holds is true for the held code and false for a different value or another code type", "[error]") {
    const alp::Error<FileError, OffsetError> error(OffsetError{.offset = 7});

    CHECK(error.holds(OffsetError{.offset = 7}));
    CHECK(!error.holds(OffsetError{.offset = 8}));
    CHECK(!error.holds(FileError::NotFound));
}

TEST_CASE("holds works in a set with an incomparable code", "[error]") {
    const alp::Error<FileError, Incomparable> file_error(FileError::NotFound);
    const alp::Error<FileError, Incomparable> incomparable_error(Incomparable{});

    CHECK(file_error.holds(FileError::NotFound));
    CHECK(!incomparable_error.holds(FileError::NotFound));
}

TEST_CASE("holds<Fs...> is true for a code of a type among Fs", "[error]") {
    SECTION("one type") {
        const alp::Error<FileError, OffsetError> error(OffsetError{.offset = 7});

        CHECK(error.holds<OffsetError>());
        CHECK(!error.holds<FileError>());
    }
    SECTION("several types") {
        const alp::Error<FileError, OffsetError, Incomparable> error(OffsetError{.offset = 7});

        CHECK(error.holds<FileError, OffsetError>());
        CHECK(!error.holds<FileError, Incomparable>());
    }
    SECTION("a set") {
        const alp::Error<FileError, OffsetError, Incomparable> error(OffsetError{.offset = 7});

        CHECK(error.holds<alp::Error<FileError, OffsetError>>());
        CHECK(!error.holds<alp::Error<FileError, Incomparable>>());
    }
    SECTION("a code without equality") {
        const alp::Error<FileError, Incomparable> error(Incomparable{});

        CHECK(error.holds<Incomparable>());
        CHECK(!error.holds<FileError>());
    }
    SECTION("an erased error") {
        const alp::Error<> erased = alp::make_error(OffsetError{.offset = 7});

        CHECK(erased.holds<OffsetError>());
        CHECK(!erased.holds<FileError>());
        CHECK(erased.holds<alp::Error<FileError, OffsetError>>());
        CHECK(!erased.holds<alp::Error<FileError, Incomparable>>());
    }
}

TEST_CASE("holds_and<Fs...> is true only for a code of a type among Fs that satisfies the predicate", "[error]") {
    const alp::Error<FileError, OffsetError> error(OffsetError{.offset = 7});

    SECTION("one type") {
        CHECK(error.holds_and<OffsetError>([](const OffsetError &c) { return c.offset == 7; }));
        CHECK(!error.holds_and<OffsetError>([](const OffsetError &c) { return c.offset == 8; }));
        CHECK(!error.holds_and<FileError>([](const FileError) { return true; }));
    }
    SECTION("several types or a set") {
        const auto is_offset_7 = overloaded{
            [](const FileError) { return false; },
            [](const OffsetError &c) { return c.offset == 7; },
        };

        CHECK(error.holds_and<FileError, OffsetError>(is_offset_7));
        CHECK(error.holds_and<alp::Error<FileError, OffsetError>>(is_offset_7));
        CHECK(!error.holds_and<FileError, OffsetError>([](const auto &) { return false; }));
    }
}

TEST_CASE("get_if points at the held code if it has that type, and is null otherwise", "[error]") {
    const alp::Error<FileError, OffsetError> error(OffsetError{.offset = 7});

    const OffsetError *offset_error = error.get_if<OffsetError>();
    REQUIRE(offset_error != nullptr);
    CHECK(offset_error->offset == 7);
    CHECK(error.get_if<FileError>() == nullptr);
}

TEST_CASE("get_if on an erased error points at the code it was erased from, and is null for another type", "[error]") {
    const alp::Error<> erased = alp::make_error(OffsetError{.offset = 7});

    const OffsetError *offset_error = erased.get_if<OffsetError>();
    REQUIRE(offset_error != nullptr);
    CHECK(offset_error->offset == 7);
    CHECK(erased.get_if<FileError>() == nullptr);
}

TEST_CASE("visit calls the branch for the held code and returns what it returns", "[error]") {
    const alp::Error<FileError, OffsetError> error(OffsetError{.offset = 7});

    const uint32_t offset = error.visit(overloaded{
        [](const FileError) -> uint32_t { return 0; },
        [](const OffsetError &code) -> uint32_t { return code.offset; },
    });
    CHECK(offset == 7);
}

TEST_CASE("visit_if<Fs...> returns what f returns for a code of a type among Fs, and nothing otherwise", "[error]") {
    const auto offset_or_zero = overloaded{
        [](const FileError) -> uint32_t { return 0; },
        [](const OffsetError &code) -> uint32_t { return code.offset; },
    };
    const auto zero = [](const auto &) -> uint32_t { return 0; };

    SECTION("one type") {
        const alp::Error<FileError, OffsetError> error(OffsetError{.offset = 7});

        CHECK(error.visit_if<OffsetError>(offset_or_zero) == 7u);
        CHECK(error.visit_if<FileError>(zero) == std::nullopt);
    }
    SECTION("several types") {
        const alp::Error<FileError, OffsetError, Incomparable> error(OffsetError{.offset = 7});

        CHECK(error.visit_if<FileError, OffsetError>(offset_or_zero) == 7u);
        CHECK(error.visit_if<FileError, Incomparable>(zero) == std::nullopt);
    }
    SECTION("a set") {
        const alp::Error<FileError, OffsetError, Incomparable> error(OffsetError{.offset = 7});

        CHECK(error.visit_if<alp::Error<FileError, OffsetError>>(offset_or_zero) == 7u);
        CHECK(error.visit_if<alp::Error<FileError, Incomparable>>(zero) == std::nullopt);
    }
    SECTION("an erased error") {
        const alp::Error<> erased = alp::make_error(OffsetError{.offset = 7});

        CHECK(erased.visit_if<OffsetError>(offset_or_zero) == 7u);
        CHECK(erased.visit_if<alp::Error<FileError, OffsetError>>(offset_or_zero) == 7u);
        CHECK(erased.visit_if<FileError>(zero) == std::nullopt);
    }
}
