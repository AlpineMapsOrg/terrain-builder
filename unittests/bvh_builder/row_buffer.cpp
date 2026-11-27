#include "../catch2_helpers.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "enumerate.h"

#include "widen/RowBuffer.h"

namespace {

std::vector<std::span<const int>> collect_rows(const widen::RowBuffer<int, uint8_t> &buffer) {
    std::vector<std::span<const int>> rows;
    for (const std::span<const int> row : buffer) {
        rows.push_back(row);
    }
    return rows;
}

}

TEST_CASE("a new row buffer holds no rows", "[widen][RowBuffer]") {
    const widen::RowBuffer<int, uint8_t> buffer;

    CHECK(buffer.size() == 0);
    CHECK(collect_rows(buffer).empty());
}

TEST_CASE("rows come back in the order they were pushed", "[widen][RowBuffer]") {
    const std::array first{1, 2, 3};
    const std::array second{4, 5};

    widen::RowBuffer<int, uint8_t> buffer;
    buffer.push_row(first);
    buffer.push_row(second);

    REQUIRE(buffer.size() == 2);

    const std::vector<std::span<const int>> rows = collect_rows(buffer);
    REQUIRE(rows.size() == 2);
    CHECK(std::ranges::equal(rows[0], first));
    CHECK(std::ranges::equal(rows[1], second));
}

TEST_CASE("an empty row is still a row", "[widen][RowBuffer]") {
    const std::array outer{7};

    widen::RowBuffer<int, uint8_t> buffer;
    buffer.push_row(outer);
    buffer.push_row(std::span<const int>{});
    buffer.push_row(outer);

    REQUIRE(buffer.size() == 3);

    const std::vector<std::span<const int>> rows = collect_rows(buffer);
    REQUIRE(rows.size() == 3);
    CHECK(rows[0].size() == 1);
    CHECK(rows[1].empty());
    CHECK(rows[2].size() == 1);
}

TEST_CASE("rows of differing lengths keep their boundaries", "[widen][RowBuffer]") {
    const std::vector<size_t> lengths{1, 4, 2, 255, 3};

    widen::RowBuffer<int, uint8_t> buffer;
    for (const auto &[index, length] : enumerate<int>(lengths)) {
        const std::vector<int> row(length, index);
        buffer.push_row(row);
    }

    REQUIRE(buffer.size() == lengths.size());

    const std::vector<std::span<const int>> rows = collect_rows(buffer);
    REQUIRE(rows.size() == lengths.size());
    for (const auto &[index, row] : enumerate<int>(rows)) {
        REQUIRE(row.size() == lengths[index]);
        REQUIRE(std::ranges::all_of(row, [index](const int value) { return value == index; }));
    }
}
