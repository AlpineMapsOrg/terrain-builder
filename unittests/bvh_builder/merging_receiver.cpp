#include "../catch2_helpers.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <iterator>
#include <numeric>
#include <optional>
#include <random>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "build/MergingReceiver.h"

namespace {

void close_all(std::vector<spsc::Sender<int>> &senders) {
    for (spsc::Sender<int> &sender : senders) {
        std::move(sender).close();
    }
}

// Pops until every source has ended.
std::vector<int> drain(auto &merging) {
    std::vector<int> values;
    while (const std::optional<int> value = merging.pop()) {
        values.push_back(*value);
    }
    return values;
}

} // namespace

TEST_CASE("a lone source streams straight through", "[build][MergingReceiver]") {
    auto [senders, merging] = make_merging_channels<int>(1, 16);

    REQUIRE(senders[0].push_batch(std::array<int, 3>{1, 2, 3}));
    close_all(senders);

    CHECK(drain(merging) == std::vector<int>{1, 2, 3});
}

TEST_CASE("two sources interleave in sorted order", "[build][MergingReceiver]") {
    auto [senders, merging] = make_merging_channels<int>(2, 16);

    REQUIRE(senders[0].push_batch(std::array<int, 3>{1, 3, 5}));
    REQUIRE(senders[1].push_batch(std::array<int, 3>{2, 4, 6}));
    close_all(senders);

    CHECK(drain(merging) == std::vector<int>{1, 2, 3, 4, 5, 6});
}

TEST_CASE("the comparator decides the merge order", "[build][MergingReceiver]") {
    auto [senders, merging] = make_merging_channels<int>(2, 16, std::greater<>{});

    REQUIRE(senders[0].push_batch(std::array<int, 3>{5, 3, 1}));
    REQUIRE(senders[1].push_batch(std::array<int, 3>{6, 4, 2}));
    close_all(senders);

    CHECK(drain(merging) == std::vector<int>{6, 5, 4, 3, 2, 1});
}

TEST_CASE("a source that ends without a value holds nothing back", "[build][MergingReceiver]") {
    auto [senders, merging] = make_merging_channels<int>(3, 16);

    REQUIRE(senders[1].push_batch(std::array<int, 2>{7, 8}));
    close_all(senders);

    CHECK(drain(merging) == std::vector<int>{7, 8});
}

TEST_CASE("sources that all end without a value merge to nothing", "[build][MergingReceiver]") {
    auto [senders, merging] = make_merging_channels<int>(3, 16);

    close_all(senders);

    CHECK(drain(merging).empty());
}

TEST_CASE("a source longer than the prefetch is drained in order", "[build][MergingReceiver]") {
    constexpr size_t count = 500;

    auto [senders, merging] = make_merging_channels<int>(1, 1024);

    std::vector<int> stream(count);
    std::iota(stream.begin(), stream.end(), 0);
    REQUIRE(senders[0].push_batch(stream));
    close_all(senders);

    CHECK(drain(merging) == stream);
}

TEST_CASE("end to end, four sorted streams merge into one", "[build][MergingReceiver]") {
    constexpr size_t source_count = 4;
    constexpr size_t per_source = 500;

    std::mt19937 rng(1234);
    std::uniform_int_distribution<int> values(0, 1000);

    std::array<std::vector<int>, source_count> streams;
    std::vector<int> expected;
    for (std::vector<int> &stream : streams) {
        std::ranges::generate_n(std::back_inserter(stream), per_source, [&] {
            return values(rng);
        });
        std::ranges::sort(stream);
        std::ranges::copy(stream, std::back_inserter(expected));
    }
    std::ranges::sort(expected);

    // A ring far smaller than a stream, so the producers spend the run under backpressure.
    auto [senders, merging] = make_merging_channels<int>(source_count, 8);

    std::vector<std::thread> producers;
    for (size_t source = 0; source < source_count; source++) {
        producers.emplace_back([&, source] {
            spsc::Sender<int> sender = std::move(senders[source]);
            for (const int value : streams[source]) {
                if (!sender.push(value)) {
                    return;
                }
            }
            std::move(sender).close();
        });
    }

    const std::vector<int> merged = drain(merging);
    for (std::thread &producer : producers) {
        producer.join();
    }

    CHECK(merged == expected);
}
