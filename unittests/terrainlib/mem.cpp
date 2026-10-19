#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>
#include <utility>

#include "mem.h"

namespace {

template <typename T>
concept Droppable = requires(T &&value) { drop(std::forward<T>(value)); };

} // namespace

TEST_CASE("drop destroys the value moved into it", "[mem]") {
    const std::shared_ptr<int> kept = std::make_shared<int>(7);
    std::shared_ptr<int> dropped = kept;
    REQUIRE(kept.use_count() == 2);

    drop(std::move(dropped));

    CHECK(kept.use_count() == 1);
}

TEST_CASE("drop only accepts a value that is moved in", "[mem]") {
    STATIC_REQUIRE(Droppable<std::string>);
    STATIC_REQUIRE(Droppable<std::string &&>);
    STATIC_REQUIRE_FALSE(Droppable<std::string &>);
    STATIC_REQUIRE_FALSE(Droppable<const std::string &>);
}
