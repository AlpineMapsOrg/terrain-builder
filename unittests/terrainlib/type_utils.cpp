#include <catch2/catch_test_macros.hpp>

#include <tuple>
#include <type_traits>

#include "type_utils.h"

namespace {

template <typename... Ts>
struct List {};

struct A {};
struct B {};
struct C {};

} // namespace

TEST_CASE("flatten_t replaces nested specializations of the same template by their arguments, at any depth", "[type_utils]") {
    STATIC_REQUIRE(std::is_same_v<flatten_t<List<>>, List<>>);
    STATIC_REQUIRE(std::is_same_v<flatten_t<List<A, B>>, List<A, B>>);
    STATIC_REQUIRE(std::is_same_v<flatten_t<List<List<A, B>, C>>, List<A, B, C>>);
    STATIC_REQUIRE(std::is_same_v<flatten_t<List<A, List<List<B>, List<>>, C>>, List<A, B, C>>);
}

TEST_CASE("flatten_t keeps a specialization of another template as one argument", "[type_utils]") {
    STATIC_REQUIRE(std::is_same_v<flatten_t<List<std::tuple<A>, B>>, List<std::tuple<A>, B>>);
}

TEST_CASE("unique_t keeps each argument once, in the order it first appears", "[type_utils]") {
    STATIC_REQUIRE(std::is_same_v<unique_t<List<A, B, A, C, B>>, List<A, B, C>>);
    STATIC_REQUIRE(std::is_same_v<unique_t<List<>>, List<>>);
}

TEST_CASE("unique_t over flatten_t removes duplicates across nesting levels", "[type_utils]") {
    STATIC_REQUIRE(std::is_same_v<unique_t<flatten_t<List<A, List<B, A>, C>>>, List<A, B, C>>);
}

TEST_CASE("is_subset_v holds when every argument of one specialization is among those of another", "[type_utils]") {
    STATIC_REQUIRE(is_subset_v<List<A, B>, List<B, C, A>>);
    STATIC_REQUIRE(is_subset_v<List<A, A>, List<A>>);
    STATIC_REQUIRE(is_subset_v<List<>, List<A>>);
    STATIC_REQUIRE(is_subset_v<List<A>, std::tuple<A, B>>);
    STATIC_REQUIRE(!is_subset_v<List<A, C>, List<A, B>>);
    STATIC_REQUIRE(!is_subset_v<List<A>, List<>>);
}

TEST_CASE("difference_t removes the arguments that are among those of another specialization, keeping the order of the rest", "[type_utils]") {
    STATIC_REQUIRE(std::is_same_v<difference_t<List<A, B, C>, List<B>>, List<A, C>>);
    STATIC_REQUIRE(std::is_same_v<difference_t<List<A, B, C>, List<C, A>>, List<B>>);
    STATIC_REQUIRE(std::is_same_v<difference_t<List<A, B>, List<>>, List<A, B>>);
    STATIC_REQUIRE(std::is_same_v<difference_t<List<A, B>, List<B, A>>, List<>>);
    STATIC_REQUIRE(std::is_same_v<difference_t<List<>, List<A>>, List<>>);
}

TEST_CASE("difference_t ignores arguments to remove that the specialization does not have", "[type_utils]") {
    STATIC_REQUIRE(std::is_same_v<difference_t<List<A, B>, List<C, B>>, List<A>>);
}

TEST_CASE("difference_t removes every occurrence of a repeated argument", "[type_utils]") {
    STATIC_REQUIRE(std::is_same_v<difference_t<List<A, B, A>, List<A>>, List<B>>);
}

TEST_CASE("difference_t takes the arguments to remove from a specialization of any template", "[type_utils]") {
    STATIC_REQUIRE(std::is_same_v<difference_t<List<A, B>, std::tuple<B>>, List<A>>);
}
