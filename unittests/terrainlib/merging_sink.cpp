#include "../catch2_helpers.h"
#include "../result_helpers.h"

#include <array>
#include <functional>
#include <string_view>
#include <utility>
#include <vector>

#include "error/Result.h"
#include "error/utils.h"
#include "sink/FnSink.h"
#include "sink/MergingSink.h"

namespace {

enum class OutputError {
    Rejected,
};

std::string_view format_as(const OutputError) {
    return "output rejected";
}

} // namespace

TEST_CASE("a lone source streams straight through", "[sink][MergingSink]") {
    std::vector<int> out;
    bool finished = false;
    auto [input] = make_merging_sink<1, int>(FnSink(
        [&out](const int value) { out.push_back(value); },
        [&finished] { finished = true; }));

    REQUIRE_VALUE(input.push(std::array{1, 2}));
    CHECK(out == std::vector<int>{1, 2});

    REQUIRE_VALUE(std::move(input).finish());
    CHECK(out == std::vector<int>{1, 2});
    CHECK(finished);
}

TEST_CASE("a silent source holds the others back until it finishes, and the last source to finish finishes the output", "[sink][MergingSink]") {
    std::vector<int> out;
    bool finished = false;
    auto [first, second] = make_merging_sink<2, int>(FnSink(
        [&out](const int value) { out.push_back(value); },
        [&finished] { finished = true; }));

    REQUIRE_VALUE(first.push(std::array{1, 2}));
    CHECK(out.empty());

    REQUIRE_VALUE(std::move(second).finish());
    CHECK(out == std::vector<int>{1, 2});
    CHECK_FALSE(finished);

    REQUIRE_VALUE(std::move(first).finish());
    CHECK(finished);
}

TEST_CASE("two sources interleave in sorted order", "[sink][MergingSink]") {
    std::vector<int> out;
    auto [first, second] = make_merging_sink<2, int>(FnSink([&out](const int value) { out.push_back(value); }));

    REQUIRE_VALUE(first.push(std::array{1}));
    REQUIRE_VALUE(second.push(std::array{2}));
    CHECK(out == std::vector<int>{1});

    REQUIRE_VALUE(first.push(std::array{3}));
    CHECK(out == std::vector<int>{1, 2});

    REQUIRE_VALUE(second.push(std::array{4}));
    CHECK(out == std::vector<int>{1, 2, 3});

    REQUIRE_VALUE(std::move(first).finish());
    CHECK(out == std::vector<int>{1, 2, 3, 4});

    REQUIRE_VALUE(std::move(second).finish());
    CHECK(out == std::vector<int>{1, 2, 3, 4});
}

TEST_CASE("the comparator decides the merge order", "[sink][MergingSink]") {
    std::vector<int> out;
    auto [first, second] = make_merging_sink<2, int>(FnSink([&out](const int value) { out.push_back(value); }), std::greater<>{});

    REQUIRE_VALUE(first.push(std::array{5}));
    REQUIRE_VALUE(second.push(std::array{9}));
    CHECK(out == std::vector<int>{9});

    REQUIRE_VALUE(second.push(std::array{7}));
    CHECK(out == std::vector<int>{9, 7});

    REQUIRE_VALUE(std::move(second).finish());
    CHECK(out == std::vector<int>{9, 7, 5});

    REQUIRE_VALUE(std::move(first).finish());
    CHECK(out == std::vector<int>{9, 7, 5});
}

TEST_CASE("an output's error comes back from the push that reached it", "[sink][MergingSink]") {
    auto [input] = make_merging_sink<1, int>(FnSink([](int) -> alp::Result<void, OutputError> {
        return alp::make_error(OutputError::Rejected);
    }));

    const alp::Error<OutputError> error = REQUIRE_ERROR(input.push(std::array{1}));

    CHECK(error.holds_code(OutputError::Rejected));
}
