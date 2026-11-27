#include "../catch2_helpers.h"
#include "node_header_helpers.h"

#include <cstddef>
#include <optional>
#include <span>
#include <tuple>
#include <vector>

#include <glm/glm.hpp>

#include "range_utils.h"

#include "frontier/layout.h"
#include "ploc/NearestSearch.h"

namespace {

// An entry holding only a unit box starting at `x` along the x axis, with `error` as both ends of its error interval.
frontier::Entry make_entry(const double x, const double error = 0.0) {
    return frontier::Entry{
        .morton = {},
        .header = make_node_header({error, error}, {glm::dvec3{x, 0.0, 0.0}, glm::dvec3{x + 1.0, 1.0, 1.0}}),
        .ref = {},
    };
}

std::vector<std::optional<size_t>> collect_partners(const std::span<const ploc::Nearest> nearest) {
    return transform_vector(nearest, [](const ploc::Nearest &n) { return n.index; });
}

std::vector<double> collect_costs(const std::span<const ploc::Nearest> nearest) {
    return transform_vector(nearest, [](const ploc::Nearest &n) { return n.cost; });
}

}

TEST_CASE("find_nearest picks the cheapest partner within the radius", "[ploc][NearestSearch]") {
    // Entries 0 and 2 touch. Entry 1 lies far off.
    const std::vector<frontier::Entry> entries{make_entry(0.0), make_entry(100.0), make_entry(1.0)};

    SECTION("at radius 1 entry 0 cannot reach entry 2") {
        ploc::NearestSearch search(1, 1.0);
        CHECK(collect_partners(search.find_nearest(entries)) == std::vector<std::optional<size_t>>{1, 2, 1});
    }
    SECTION("at radius 2 entries 0 and 2 pick each other") {
        ploc::NearestSearch search(2, 1.0);
        CHECK(collect_partners(search.find_nearest(entries)) == std::vector<std::optional<size_t>>{2, 2, 0});
    }
}

TEST_CASE("find_nearest reports each partner's cost", "[ploc][NearestSearch]") {
    const std::vector<frontier::Entry> entries{make_entry(0.0), make_entry(100.0), make_entry(1.0)};

    // A box of length x costs 2 + 4x.
    // The box around entries 0 and 2 is 2 long, and the one around entries 1 and 2 is 100 long.
    ploc::NearestSearch search(2, 1.0);
    CHECK(collect_costs(search.find_nearest(entries)) == std::vector<double>{10.0, 402.0, 10.0});
}

TEST_CASE("find_nearest keeps the first of equal partners", "[ploc][NearestSearch]") {
    // Entry 1 lies as far from entry 0 as from entry 2.
    const std::vector<frontier::Entry> entries{make_entry(0.0), make_entry(2.0), make_entry(4.0)};

    ploc::NearestSearch search(1, 1.0);
    CHECK(collect_partners(search.find_nearest(entries)) == std::vector<std::optional<size_t>>{1, 0, 1});
}

TEST_CASE("find_nearest scales the error by the weight", "[ploc][NearestSearch]") {
    // Entry 1 lies nearest to entry 0 but differs from it in error by 10.
    // Joining entries 0 and 2 costs 18 at any weight.
    // Joining entries 0 and 1 costs 14 at weight 0.05 and 26 at weight 0.2.
    const std::vector<frontier::Entry> entries{make_entry(0.0), make_entry(1.0, 10.0), make_entry(3.0)};

    SECTION("at weight 0.05 entry 0 picks entry 1") {
        ploc::NearestSearch search(2, 0.05);
        CHECK(collect_partners(search.find_nearest(entries)) == std::vector<std::optional<size_t>>{1, 0, 0});
    }
    SECTION("at weight 0.2 entry 0 picks entry 2") {
        ploc::NearestSearch search(2, 0.2);
        CHECK(collect_partners(search.find_nearest(entries)) == std::vector<std::optional<size_t>>{2, 0, 0});
    }
}

TEST_CASE("find_nearest gives a lone entry no partner", "[ploc][NearestSearch]") {
    const std::vector<frontier::Entry> entries{make_entry(0.0)};

    ploc::NearestSearch search(2, 1.0);
    CHECK(collect_partners(search.find_nearest(entries)) == std::vector<std::optional<size_t>>{std::nullopt});
}

TEST_CASE("find_nearest forgets the previous call", "[ploc][NearestSearch]") {
    const std::vector<frontier::Entry> three{make_entry(0.0), make_entry(100.0), make_entry(1.0)};
    const std::vector<frontier::Entry> two{make_entry(0.0), make_entry(5.0)};

    ploc::NearestSearch search(2, 1.0);
    std::ignore = search.find_nearest(three);
    CHECK(collect_partners(search.find_nearest(two)) == std::vector<std::optional<size_t>>{1, 0});
}

TEST_CASE("find_mutual_partner needs both to agree", "[ploc][NearestSearch]") {
    // Entry 0 picks entry 1, entries 1 and 2 pick each other, and entry 3 picks nobody.
    const std::vector<ploc::Nearest> nearest{{.index = 1}, {.index = 2}, {.index = 1}, {.index = std::nullopt}};

    CHECK(ploc::find_mutual_partner(nearest, 0) == std::nullopt);
    CHECK(ploc::find_mutual_partner(nearest, 1) == std::optional<size_t>{2});
    CHECK(ploc::find_mutual_partner(nearest, 2) == std::optional<size_t>{1});
    CHECK(ploc::find_mutual_partner(nearest, 3) == std::nullopt);
}
