/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Martin Braunsperger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#include "../catch2_helpers.h"
#include "containers/SegmentedBuffer.h"

TEST_CASE("SegmentedBuffer default construction") {
    SegmentedBuffer<int> buf;

    CHECK(buf.segment_count() == 1);
    CHECK(buf.total_size() == 0);
    CHECK(buf.segment_size(0) == 0);
}

TEST_CASE("SegmentedBuffer construct from vector") {
    std::vector<int> data = {10, 20, 30};
    SegmentedBuffer<int> buf(data);

    REQUIRE(buf.segment_count() == 1);
    CHECK(buf.total_size() == 3);
    REQUIRE(buf.segment_size(0) == 3);
    CHECK(buf(0, 0) == 10);
    CHECK(buf(0, 1) == 20);
    CHECK(buf(0, 2) == 30);
}

TEST_CASE("SegmentedBuffer push_to_last_segment") {
    SegmentedBuffer<int> buf;

    buf.push_to_last_segment(1);
    buf.push_to_last_segment(2);
    buf.push_to_last_segment(3);

    REQUIRE(buf.segment_count() == 1);
    REQUIRE(buf.segment_size(0) == 3);
    CHECK(buf(0, 0) == 1);
    CHECK(buf(0, 1) == 2);
    CHECK(buf(0, 2) == 3);
}

TEST_CASE("SegmentedBuffer start_new_segment then push_to_last_segment") {
    SegmentedBuffer<int> buf;

    buf.push_to_last_segment(10);
    buf.push_to_last_segment(20);

    buf.start_new_segment();

    buf.push_to_last_segment(30);
    buf.push_to_last_segment(40);
    buf.push_to_last_segment(50);

    REQUIRE(buf.segment_count() == 2);
    REQUIRE(buf.segment_size(0) == 2);
    REQUIRE(buf.segment_size(1) == 3);
    CHECK(buf.total_size() == 5);
    CHECK(buf(0, 0) == 10);
    CHECK(buf(0, 1) == 20);
    CHECK(buf(1, 0) == 30);
    CHECK(buf(1, 1) == 40);
    CHECK(buf(1, 2) == 50);
}

TEST_CASE("SegmentedBuffer push_new_segment with size") {
    SegmentedBuffer<int> buf;

    buf.push_new_segment(4, 99);

    REQUIRE(buf.segment_count() == 1);
    REQUIRE(buf.segment_size(0) == 4);
    CHECK(buf.total_size() == 4);
    CHECK(buf(0, 0) == 99);
    CHECK(buf(0, 1) == 99);
    CHECK(buf(0, 2) == 99);
    CHECK(buf(0, 3) == 99);
}

TEST_CASE("SegmentedBuffer resize_last_segment after start_new_segment") {
    SegmentedBuffer<int> buf;

    buf.push_new_segment(2, 1);
    buf.push_new_segment(3, 2);

    REQUIRE(buf.segment_count() == 2);
    REQUIRE(buf.segment_size(0) == 2);
    REQUIRE(buf.segment_size(1) == 3);
    REQUIRE(buf.total_size() == 5);
    CHECK(buf(0, 0) == 1);
    CHECK(buf(0, 1) == 1);
    CHECK(buf(1, 0) == 2);
    CHECK(buf(1, 1) == 2);
    CHECK(buf(1, 2) == 2);
}

TEST_CASE("SegmentedBuffer push_new_segment from range") {
    SegmentedBuffer<int> buf;

    buf.push_to_last_segment(1);

    std::vector<int> data = {100, 200, 300};
    buf.push_new_segment(data);

    REQUIRE(buf.segment_count() == 2);
    REQUIRE(buf.segment_size(0) == 1);
    REQUIRE(buf.segment_size(1) == 3);
    REQUIRE(buf.total_size() == 4);
    CHECK(buf(0, 0) == 1);
    CHECK(buf(1, 0) == 100);
    CHECK(buf(1, 1) == 200);
    CHECK(buf(1, 2) == 300);
}

TEST_CASE("SegmentedBuffer init with segment sizes") {
    SegmentedBuffer<int> buf;

    std::vector<size_t> sizes = {2, 3, 1};
    buf.init(sizes, 42);

    REQUIRE(buf.segment_count() == 3);
    REQUIRE(buf.segment_size(0) == 2);
    REQUIRE(buf.segment_size(1) == 3);
    REQUIRE(buf.segment_size(2) == 1);
    REQUIRE(buf.total_size() == 6);
    CHECK(buf(0, 0) == 42);
    CHECK(buf(1, 2) == 42);
    CHECK(buf(2, 0) == 42);
}

TEST_CASE("SegmentedBuffer operator() access across segments") {
    SegmentedBuffer<int> buf;

    std::vector<size_t> sizes = {2, 3};
    buf.init(sizes, 0);

    buf(0, 0) = 10;
    buf(0, 1) = 20;
    buf(1, 0) = 30;
    buf(1, 1) = 40;
    buf(1, 2) = 50;

    CHECK(buf(0, 0) == 10);
    CHECK(buf(0, 1) == 20);
    CHECK(buf(1, 0) == 30);
    CHECK(buf(1, 1) == 40);
    CHECK(buf(1, 2) == 50);
}

TEST_CASE("SegmentedBuffer segment() span view") {
    SegmentedBuffer<int> buf;

    buf.push_to_last_segment(5);
    buf.push_to_last_segment(6);

    buf.start_new_segment();

    buf.push_to_last_segment(7);

    auto seg0 = buf.segment(0);
    auto seg1 = buf.segment(1);

    REQUIRE(seg0.size() == 2);
    CHECK(seg0[0] == 5);
    CHECK(seg0[1] == 6);

    REQUIRE(seg1.size() == 1);
    CHECK(seg1[0] == 7);
}

TEST_CASE("SegmentedBuffer flat() view") {
    SegmentedBuffer<int> buf;

    buf.push_to_last_segment(1);
    buf.push_to_last_segment(2);

    buf.start_new_segment();

    buf.push_to_last_segment(3);
    buf.push_to_last_segment(4);
    buf.push_to_last_segment(5);

    auto flat = buf.flat();

    REQUIRE(flat.size() == 5);
    CHECK(flat[0] == 1);
    CHECK(flat[1] == 2);
    CHECK(flat[2] == 3);
    CHECK(flat[3] == 4);
    CHECK(flat[4] == 5);
}

TEST_CASE("SegmentedBuffer reset") {
    SegmentedBuffer<int> buf;

    buf.push_to_last_segment(1);
    buf.push_to_last_segment(2);
    buf.start_new_segment();
    buf.push_to_last_segment(3);

    REQUIRE(buf.segment_count() == 2);
    REQUIRE(buf.total_size() == 3);

    buf.reset();

    REQUIRE(buf.segment_count() == 1);
    REQUIRE(buf.total_size() == 0);
    REQUIRE(buf.segment_size(0) == 0);
}

TEST_CASE("SegmentedBuffer multiple segments with different sizes and last_segment") {
    SegmentedBuffer<int> buf;

    buf.push_to_last_segment(10);

    buf.start_new_segment();
    buf.push_to_last_segment(20);
    buf.push_to_last_segment(21);

    buf.start_new_segment();
    buf.push_to_last_segment(30);
    buf.push_to_last_segment(31);
    buf.push_to_last_segment(32);

    REQUIRE(buf.segment_count() == 3);
    REQUIRE(buf.segment_size(0) == 1);
    REQUIRE(buf.segment_size(1) == 2);
    REQUIRE(buf.segment_size(2) == 3);
    REQUIRE(buf.total_size() == 6);

    auto last = buf.last_segment();
    REQUIRE(last.size() == 3);
    CHECK(last[0] == 30);
    CHECK(last[1] == 31);
    CHECK(last[2] == 32);
}

// A caller starting a segment per item must get one segment per item, even where an item
// contributes nothing. The leading placeholder segment used to swallow the second start.
TEST_CASE("SegmentedBuffer keeps empty segments") {
    SegmentedBuffer<int> buf;

    buf.start_new_segment(); // first item, contributes nothing
    buf.start_new_segment(); // second item
    buf.push_to_last_segment(10);
    buf.start_new_segment(); // third item, contributes nothing
    buf.start_new_segment(); // fourth item
    buf.push_to_last_segment(20);

    REQUIRE(buf.segment_count() == 4);
    CHECK(buf.segment_size(0) == 0);
    CHECK(buf.segment_size(1) == 1);
    CHECK(buf.segment_size(2) == 0);
    CHECK(buf.segment_size(3) == 1);
    CHECK(buf.total_size() == 2);
}

TEST_CASE("SegmentedBuffer keeps empty segments after reset") {
    SegmentedBuffer<int> buf;
    buf.start_new_segment();
    buf.push_to_last_segment(1);
    buf.reset();

    buf.start_new_segment();
    buf.start_new_segment();

    REQUIRE(buf.segment_count() == 2);
    CHECK(buf.segment_size(0) == 0);
    CHECK(buf.segment_size(1) == 0);
}

// init and the vector constructor install real segments, so a later start_new_segment
// must append rather than adopt one of them.
TEST_CASE("SegmentedBuffer start_new_segment after init appends") {
    SegmentedBuffer<int> buf;
    const std::vector<size_t> sizes{2, 3};
    buf.init(sizes);

    buf.start_new_segment();
    buf.push_to_last_segment(9);

    REQUIRE(buf.segment_count() == 3);
    CHECK(buf.segment_size(2) == 1);
}

TEST_CASE("SegmentedBuffer start_new_segment after the vector constructor appends") {
    SegmentedBuffer<int> buf(std::vector<int>{1, 2, 3});

    buf.start_new_segment();
    buf.push_to_last_segment(9);

    REQUIRE(buf.segment_count() == 2);
    CHECK(buf.segment_size(0) == 3);
    CHECK(buf.segment_size(1) == 1);
}

TEST_CASE("SegmentedBuffer pop_segment drops the last segment and its elements") {
    SegmentedBuffer<int> buf;

    buf.push_new_segment(std::vector<int>{1, 2});
    buf.push_new_segment(std::vector<int>{3, 4, 5});

    buf.pop_segment();

    REQUIRE(buf.segment_count() == 1);
    CHECK(buf.total_size() == 2);
    CHECK(buf.segment_size(0) == 2);
    CHECK(buf(0, 0) == 1);
}

TEST_CASE("SegmentedBuffer pop_segments drops several at once") {
    SegmentedBuffer<int> buf;

    buf.push_new_segment(std::vector<int>{1});
    buf.push_new_segment(std::vector<int>{2, 3});
    buf.push_new_segment(std::vector<int>{4, 5, 6});

    buf.pop_segments(2);

    REQUIRE(buf.segment_count() == 1);
    CHECK(buf.total_size() == 1);
    CHECK(buf(0, 0) == 1);
}

TEST_CASE("SegmentedBuffer popping every segment leaves it as constructed") {
    SegmentedBuffer<int> buf;

    buf.push_new_segment(std::vector<int>{1, 2});
    buf.push_new_segment(std::vector<int>{3});

    buf.pop_segments(buf.segment_count());

    REQUIRE(buf.segment_count() == 1);
    CHECK(buf.total_size() == 0);
    CHECK(buf.segment_size(0) == 0);
}

TEST_CASE("SegmentedBuffer pushes after popping every segment do not leave an empty one") {
    SegmentedBuffer<int> buf;

    buf.push_new_segment(std::vector<int>{1, 2});
    buf.pop_segment();
    buf.push_new_segment(std::vector<int>{7, 8});

    REQUIRE(buf.segment_count() == 1);
    CHECK(buf.total_size() == 2);
    CHECK(buf(0, 0) == 7);
}

TEST_CASE("SegmentedBuffer push_empty_segment adds a segment holding nothing") {
    SegmentedBuffer<int> buf;

    buf.push_new_segment(std::vector<int>{1, 2});
    buf.push_empty_segment();

    REQUIRE(buf.segment_count() == 2);
    CHECK(buf.segment_size(1) == 0);
    CHECK(buf.total_size() == 2);
}

TEST_CASE("SegmentedBuffer keeps empty segments when popping") {
    SegmentedBuffer<int> buf;

    buf.push_new_segment(std::span<const int>{});
    buf.push_new_segment(std::vector<int>{1});
    buf.push_new_segment(std::span<const int>{});

    buf.pop_segment();

    REQUIRE(buf.segment_count() == 2);
    CHECK(buf.segment_size(0) == 0);
    CHECK(buf.segment_size(1) == 1);
}
