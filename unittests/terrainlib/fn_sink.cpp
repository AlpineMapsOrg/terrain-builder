#include "../result_helpers.h"

#include <concepts>
#include <string_view>
#include <utility>
#include <vector>

#include "error/Infallible.h"
#include "error/Result.h"
#include "error/utils.h"
#include "sink/FnSink.h"

namespace {

enum class OutputError {
    Rejected,
};

std::string_view format_as(const OutputError) {
    return "output rejected";
}

} // namespace

TEST_CASE("FnSink passes on its callbacks' results", "[sink][FnSink]") {
    SECTION("push") {
        std::vector<int> out;
        FnSink infallible([&out](const int value) { out.push_back(value); });
        FnSink fallible([](int) -> alp::Result<void, OutputError> { return alp::make_error(OutputError::Rejected); });

        STATIC_REQUIRE(std::same_as<decltype(infallible.push(3)), alp::Result<void, alp::Infallible>>);
        REQUIRE_VALUE(infallible.push(3));
        CHECK(out == std::vector<int>{3});
        CHECK(REQUIRE_ERROR(fallible.push(3)).code() == OutputError::Rejected);
    }

    SECTION("finish") {
        bool finished = false;
        FnSink infallible([](int) {}, [&finished] { finished = true; });
        FnSink fallible([](int) {}, []() -> alp::Result<void, OutputError> { return alp::make_error(OutputError::Rejected); });

        STATIC_REQUIRE(std::same_as<decltype(std::move(infallible).finish()), alp::Result<void, alp::Infallible>>);
        REQUIRE_VALUE(std::move(infallible).finish());
        CHECK(finished);
        CHECK(REQUIRE_ERROR(std::move(fallible).finish()).code() == OutputError::Rejected);
    }
}
