#include "../catch2_helpers.h"

#include <cstddef>
#include <cstdint>
#include <expected>

#include "storage/BoundedCache.h"

namespace {

struct ReadFailed {
    bool operator==(const ReadFailed &) const = default;
};

using Cache = BoundedCache<uint32_t, uint32_t>;

// A read that always yields `value` at `cost`.
auto make_read(const uint32_t value, const size_t cost) {
    return [value, cost]() -> std::expected<Cache::Entry, ReadFailed> {
        return Cache::Entry{.value = value, .cost = cost};
    };
}

}

TEST_CASE("a resident value is returned without being read again", "[storage][BoundedCache]") {
    Cache cache(10);
    uint32_t reads = 0;
    const auto read = [&reads]() -> std::expected<Cache::Entry, ReadFailed> {
        reads++;
        return Cache::Entry{.value = 7, .cost = 1};
    };

    REQUIRE(cache.get_or_try_insert_with(0, read).has_value());
    const auto result = cache.get_or_try_insert_with(0, read);
    REQUIRE(result.has_value());

    CHECK(result->get() == 7);
    CHECK(reads == 1);
    CHECK(cache.count_misses() == 1);
}

TEST_CASE("a resident value is handed out without being copied", "[storage][BoundedCache]") {
    Cache cache(10);

    const auto first_result = cache.get_or_try_insert_with(0, make_read(7, 1));
    REQUIRE(first_result.has_value());
    const auto second_result = cache.get_or_try_insert_with(0, make_read(7, 1));
    REQUIRE(second_result.has_value());

    CHECK(&first_result->get() == &second_result->get());
}

TEST_CASE("the least recently used value is evicted once the costs exceed the budget", "[storage][BoundedCache]") {
    Cache cache(10);

    REQUIRE(cache.get_or_try_insert_with(0, make_read(0, 4)).has_value());
    REQUIRE(cache.get_or_try_insert_with(1, make_read(1, 4)).has_value());
    // Reading 0 again makes 1 the older of the two, so admitting 2 must drop 1.
    REQUIRE(cache.get_or_try_insert_with(0, make_read(0, 4)).has_value());
    REQUIRE(cache.get_or_try_insert_with(2, make_read(2, 4)).has_value());
    REQUIRE(cache.count_misses() == 3);
    CHECK(cache.count_resident() == 2);

    REQUIRE(cache.get_or_try_insert_with(0, make_read(0, 4)).has_value());
    CHECK(cache.count_misses() == 3);

    REQUIRE(cache.get_or_try_insert_with(1, make_read(1, 4)).has_value());
    CHECK(cache.count_misses() == 4);
}

TEST_CASE("a value over budget on its own is still admitted, evicting everything else", "[storage][BoundedCache]") {
    Cache cache(10);

    REQUIRE(cache.get_or_try_insert_with(0, make_read(0, 4)).has_value());
    REQUIRE(cache.get_or_try_insert_with(1, make_read(1, 4)).has_value());
    const auto result = cache.get_or_try_insert_with(2, make_read(2, 20));
    REQUIRE(result.has_value());

    CHECK(result->get() == 2);
    CHECK(cache.count_resident() == 1);
}

TEST_CASE("a failed read caches nothing and returns the error", "[storage][BoundedCache]") {
    Cache cache(10);
    const auto fail = []() -> std::expected<Cache::Entry, ReadFailed> {
        return std::unexpected(ReadFailed{});
    };

    const auto failed_result = cache.get_or_try_insert_with(0, fail);
    REQUIRE(!failed_result.has_value());
    CHECK(failed_result.error() == ReadFailed{});
    CHECK(cache.count_resident() == 0);

    const auto result = cache.get_or_try_insert_with(0, make_read(7, 1));
    REQUIRE(result.has_value());
    CHECK(result->get() == 7);
}
