#include <catch2/catch_test_macros.hpp>

#include <string_view>

#include "error/Result.h"
#include "sink/FnSink.h"
#include "sink/Sink.h"
#include "sink/VoidSink.h"

namespace {

enum class OutputError {
    Rejected,
};

std::string_view format_as(const OutputError) {
    return "output rejected";
}

enum class OtherError {
    Broken,
};

std::string_view format_as(const OtherError) {
    return "broken";
}

} // namespace

TEST_CASE("A sink fails with at most its codes", "[sink][Sink]") {
    using FallibleSink = decltype(FnSink([](int) -> alp::Result<void, OutputError> { return {}; }));

    STATIC_REQUIRE(Sink<VoidSink, int, OutputError>);
    STATIC_REQUIRE(Sink<FallibleSink, int, OutputError>);
    STATIC_REQUIRE(!Sink<FallibleSink, int, OtherError>);
}
