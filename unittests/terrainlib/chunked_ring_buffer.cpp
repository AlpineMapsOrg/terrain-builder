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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <optional>
#include <thread>
#include <vector>

#include "containers/ChunkedRingBuffer.h"

namespace {

using Buffer = ChunkedRingBuffer<uint32_t>;

// Elements are numbered from one across the whole stream, so a zero marks untouched memory.
uint32_t value_at(const size_t index) {
    return static_cast<uint32_t>(index + 1);
}

void fill_chunk(const std::span<uint32_t> data, const size_t index, const size_t size) {
    for (size_t i = 0; i < size; i++) {
        data[i] = value_at(index * data.size() + i);
    }
}

void write_chunk(Buffer &buffer, const size_t index) {
    auto lease = buffer.rent_for_writing();
    REQUIRE(lease.has_value());

    fill_chunk(lease->span(), index, lease->span().size());
    std::move(*lease).commit();
}

void write_final_chunk(Buffer &buffer, const size_t index, const size_t size) {
    auto lease = buffer.rent_for_writing();
    REQUIRE(lease.has_value());

    fill_chunk(lease->span(), index, size);
    std::move(*lease).commit_final(size);
}

// Writes chunk `index` without ever blocking, so a ring that refuses it fails the test.
void try_write_chunk(Buffer &buffer, const size_t index) {
    std::expected<Buffer::WriteLease, Buffer::RentError> lease = buffer.try_rent_for_writing();
    REQUIRE(lease.has_value());

    fill_chunk(lease->span(), index, lease->span().size());
    std::move(*lease).commit();
}

// Ends the stream at the chunk before `index`, as a producer that cannot continue would.
void abandon_chunk(Buffer &buffer, const size_t index) {
    auto lease = buffer.rent_for_writing();
    REQUIRE(lease.has_value());
    REQUIRE(lease->chunk_index() == index);

    std::move(*lease).abandon();
}

// Reads and immediately retires the next chunk.
void skip_chunk(Buffer &buffer, const size_t index) {
    const auto lease = buffer.rent_for_reading();
    REQUIRE(lease.has_value());
    REQUIRE(lease->chunk_index() == index);
}

} // namespace

TEST_CASE("ChunkedRingBuffer returns the elements that were written", "[chunked_ring_buffer]") {
    Buffer buffer(4, 4, 0);
    write_final_chunk(buffer, 0, 4);

    const auto lease = buffer.rent_for_reading();
    REQUIRE(lease.has_value());
    CHECK(lease->chunk_index() == 0);
    CHECK(lease->view().size() == 4);
    CHECK(std::ranges::equal(lease->view().data(), std::vector<uint32_t>{1, 2, 3, 4}));
}

TEST_CASE("ChunkedRingBuffer gives an interior chunk its neighbours as halo", "[chunked_ring_buffer]") {
    Buffer buffer(4, 8, 2);
    write_chunk(buffer, 0);
    write_chunk(buffer, 1);
    write_final_chunk(buffer, 2, 4);

    skip_chunk(buffer, 0);

    const auto lease = buffer.rent_for_reading();
    REQUIRE(lease.has_value());
    CHECK(lease->view().left_padding() == 2);
    CHECK(lease->view().right_padding() == 2);
    CHECK(lease->view().size() == 4);
    CHECK(std::ranges::equal(lease->view().span(), std::vector<uint32_t>{3, 4, 5, 6, 7, 8, 9, 10}));
    CHECK(std::ranges::equal(lease->view().data(), std::vector<uint32_t>{5, 6, 7, 8}));
    CHECK(lease->view().data().data() == lease->view().span().data() + lease->view().left_padding());
}

TEST_CASE("ChunkedRingBuffer gives the first chunk no left halo", "[chunked_ring_buffer]") {
    Buffer buffer(4, 8, 2);
    write_chunk(buffer, 0);
    write_final_chunk(buffer, 1, 4);

    const auto lease = buffer.rent_for_reading();
    REQUIRE(lease.has_value());
    CHECK(lease->view().left_padding() == 0);
    CHECK(lease->view().right_padding() == 2);
    CHECK(std::ranges::equal(lease->view().span(), std::vector<uint32_t>{1, 2, 3, 4, 5, 6}));
}

TEST_CASE("ChunkedRingBuffer gives the final chunk no right halo", "[chunked_ring_buffer]") {
    Buffer buffer(4, 8, 2);
    write_chunk(buffer, 0);
    write_final_chunk(buffer, 1, 3);

    skip_chunk(buffer, 0);

    const auto lease = buffer.rent_for_reading();
    REQUIRE(lease.has_value());
    CHECK(lease->view().left_padding() == 2);
    CHECK(lease->view().right_padding() == 0);
    CHECK(lease->view().size() == 3);
    CHECK(std::ranges::equal(lease->view().span(), std::vector<uint32_t>{3, 4, 5, 6, 7}));
}

TEST_CASE("ChunkedRingBuffer gives a single chunk stream no halo at all", "[chunked_ring_buffer]") {
    Buffer buffer(4, 4, 2);
    write_final_chunk(buffer, 0, 3);

    const auto lease = buffer.rent_for_reading();
    REQUIRE(lease.has_value());
    CHECK(lease->view().left_padding() == 0);
    CHECK(lease->view().right_padding() == 0);
    CHECK(std::ranges::equal(lease->view().span(), std::vector<uint32_t>{1, 2, 3}));
}

TEST_CASE("ChunkedRingBuffer shortens the halo before a final chunk shorter than the padding", "[chunked_ring_buffer]") {
    Buffer buffer(4, 8, 3);
    write_chunk(buffer, 0);
    write_final_chunk(buffer, 1, 1);

    const auto lease = buffer.rent_for_reading();
    REQUIRE(lease.has_value());
    CHECK(lease->chunk_index() == 0);
    CHECK(lease->view().right_padding() == 1);
    CHECK(std::ranges::equal(lease->view().span(), std::vector<uint32_t>{1, 2, 3, 4, 5}));
}

TEST_CASE("ChunkedRingBuffer keeps the halo of the last slot across a wrap", "[chunked_ring_buffer]") {
    Buffer buffer(4, 4, 2);
    try_write_chunk(buffer, 0);
    try_write_chunk(buffer, 1);
    try_write_chunk(buffer, 2);

    skip_chunk(buffer, 0);
    try_write_chunk(buffer, 3);
    skip_chunk(buffer, 1);

    // Chunk 4 wraps into the first slot, which is where both halo copies are made.
    try_write_chunk(buffer, 4);
    skip_chunk(buffer, 2);

    // Chunk 3 sits in the last slot, so its right halo is the copy after the ring.
    const auto lease = buffer.rent_for_reading();
    REQUIRE(lease.has_value());
    CHECK(lease->chunk_index() == 3);
    CHECK(std::ranges::equal(lease->view().span(), std::vector<uint32_t>{11, 12, 13, 14, 15, 16, 17, 18}));
}

TEST_CASE("ChunkedRingBuffer keeps the halo of the first slot across a wrap", "[chunked_ring_buffer]") {
    Buffer buffer(4, 4, 2);
    try_write_chunk(buffer, 0);
    try_write_chunk(buffer, 1);
    try_write_chunk(buffer, 2);

    skip_chunk(buffer, 0);
    try_write_chunk(buffer, 3);
    skip_chunk(buffer, 1);
    try_write_chunk(buffer, 4);
    skip_chunk(buffer, 2);
    skip_chunk(buffer, 3);
    write_final_chunk(buffer, 5, 4);

    // Chunk 4 sits in the first slot, so its left halo is the copy before the ring.
    const auto lease = buffer.rent_for_reading();
    REQUIRE(lease.has_value());
    CHECK(lease->chunk_index() == 4);
    CHECK(std::ranges::equal(lease->view().span(), std::vector<uint32_t>{15, 16, 17, 18, 19, 20, 21, 22}));
}

TEST_CASE("ChunkedRingBuffer reports the end of the stream to every later reader", "[chunked_ring_buffer]") {
    Buffer buffer(4, 4, 1);
    write_chunk(buffer, 0);
    write_final_chunk(buffer, 1, 2);

    skip_chunk(buffer, 0);
    skip_chunk(buffer, 1);

    CHECK(!buffer.rent_for_reading().has_value());
    CHECK(!buffer.rent_for_reading().has_value());
}

TEST_CASE("ChunkedRingBuffer refuses to write to a completed stream", "[chunked_ring_buffer]") {
    Buffer buffer(4, 4, 1);
    write_final_chunk(buffer, 0, 4);

    CHECK(!buffer.rent_for_writing().has_value());
}

TEST_CASE("ChunkedRingBuffer ends the stream at an abandoned chunk", "[chunked_ring_buffer]") {
    Buffer buffer(4, 4, 2);
    write_chunk(buffer, 0);
    write_chunk(buffer, 1);
    abandon_chunk(buffer, 2);

    skip_chunk(buffer, 0);

    // Chunk 1 is the last one the stream ever had, so it gets no right halo.
    const auto last = buffer.rent_for_reading();
    REQUIRE(last.has_value());
    CHECK(last->chunk_index() == 1);
    CHECK(last->view().size() == 4);
    CHECK(last->view().right_padding() == 0);
    CHECK(std::ranges::equal(last->view().data(), std::vector<uint32_t>{5, 6, 7, 8}));

    CHECK(!buffer.rent_for_reading().has_value());
    CHECK(!buffer.rent_for_writing().has_value());
}

TEST_CASE("ChunkedRingBuffer ends an empty stream when the first chunk is abandoned", "[chunked_ring_buffer]") {
    Buffer buffer(4, 4, 2);
    abandon_chunk(buffer, 0);

    CHECK(!buffer.rent_for_reading().has_value());
    CHECK(!buffer.rent_for_writing().has_value());
}

TEST_CASE("ChunkedRingBuffer reports how the stream ended", "[chunked_ring_buffer]") {
    Buffer completed(4, 4, 2);
    CHECK(completed.stream_state() == Buffer::StreamState::Running);
    write_final_chunk(completed, 0, 4);
    CHECK(completed.stream_state() == Buffer::StreamState::Completed);

    Buffer aborted(4, 4, 2);
    write_chunk(aborted, 0);
    abandon_chunk(aborted, 1);
    CHECK(aborted.stream_state() == Buffer::StreamState::Aborted);
}

TEST_CASE("ChunkedRingBuffer holds back a chunk whose successor is still being written", "[chunked_ring_buffer]") {
    Buffer buffer(4, 8, 2);
    write_chunk(buffer, 0);

    // Two leases outstanding, and the later one completes the stream first.
    std::expected<Buffer::WriteLease, Buffer::RentError> first = buffer.try_rent_for_writing();
    REQUIRE(first.has_value());
    std::expected<Buffer::WriteLease, Buffer::RentError> second = buffer.try_rent_for_writing();
    REQUIRE(second.has_value());
    REQUIRE(first->chunk_index() == 1);
    REQUIRE(second->chunk_index() == 2);

    fill_chunk(second->span(), 2, 4);
    std::move(*second).commit_final(4);

    // Chunk 0's right halo is chunk 1, which nobody has written yet.
    const std::expected<Buffer::ReadLease, Buffer::RentError> refused = buffer.try_rent_for_reading();
    REQUIRE(!refused.has_value());
    CHECK(refused.error() == Buffer::RentError::WouldBlock);

    fill_chunk(first->span(), 1, 4);
    std::move(*first).commit();

    skip_chunk(buffer, 0);
    skip_chunk(buffer, 1);
    const auto last = buffer.rent_for_reading();
    REQUIRE(last.has_value());
    CHECK(last->chunk_index() == 2);
    CHECK(!buffer.rent_for_reading().has_value());
}

TEST_CASE("ChunkedRingBuffer passes a whole stream through a ring that holds it", "[chunked_ring_buffer]") {
    constexpr size_t chunk_size = 4;
    constexpr size_t chunk_count = 6;
    constexpr size_t final_size = 2;

    Buffer buffer(chunk_size, 8, 2);
    for (size_t index = 0; index + 1 < chunk_count; index++) {
        write_chunk(buffer, index);
    }
    write_final_chunk(buffer, chunk_count - 1, final_size);

    std::vector<uint32_t> read;
    size_t expected_index = 0;
    while (const auto lease = buffer.rent_for_reading()) {
        REQUIRE(lease->chunk_index() == expected_index);
        std::ranges::copy(lease->view().data(), std::back_inserter(read));
        expected_index++;
    }

    CHECK(expected_index == chunk_count);
    CHECK(read.size() == (chunk_count - 1) * chunk_size + final_size);
    for (size_t i = 0; i < read.size(); i++) {
        REQUIRE(read[i] == value_at(i));
    }
}

TEST_CASE("ChunkedRingBuffer passes a stream from a producer to three consumers", "[chunked_ring_buffer]") {
    constexpr size_t chunk_size = 32;
    constexpr size_t chunk_count = 500;
    constexpr size_t padding = 4;
    constexpr size_t final_size = 7;

    Buffer buffer(chunk_size, 4, padding);

    // Catch2 assertions are not thread safe, so the consumers only record what they saw.
    std::vector<std::atomic<uint32_t>> times_read(chunk_count);
    std::atomic<size_t> mismatches = 0;

    std::thread producer([&] {
        for (size_t index = 0; index + 1 < chunk_count; index++) {
            write_chunk(buffer, index);
        }
        write_final_chunk(buffer, chunk_count - 1, final_size);
    });

    std::vector<std::thread> consumers;
    for (size_t i = 0; i < 3; i++) {
        consumers.emplace_back([&] {
            while (const auto lease = buffer.rent_for_reading()) {
                const size_t index = lease->chunk_index();
                const bool is_final = index + 1 == chunk_count;
                const size_t successor_size = index + 2 == chunk_count ? final_size : chunk_size;

                bool matches = lease->view().size() == (is_final ? final_size : chunk_size)
                    && lease->view().left_padding() == (index == 0 ? 0 : padding)
                    && lease->view().right_padding() == (is_final ? 0 : std::min(padding, successor_size))
                    && lease->view().span().size() == lease->view().left_padding() + lease->view().size() + lease->view().right_padding();

                const size_t first = index * chunk_size - lease->view().left_padding();
                for (size_t j = 0; matches && j < lease->view().span().size(); j++) {
                    matches = lease->view().span()[j] == value_at(first + j);
                }

                if (!matches) {
                    mismatches++;
                }
                times_read[index]++;
            }
        });
    }

    producer.join();
    for (std::thread &consumer : consumers) {
        consumer.join();
    }

    CHECK(mismatches == 0);
    CHECK(std::ranges::all_of(times_read, [](const auto &t) { return t == 1; }));
}
TEST_CASE("ChunkedRingBuffer releases a parked writer when the stream ends", "[chunked_ring_buffer]") {
    // A ring of three holds two chunks before a writer has to wait for a reader.
    Buffer buffer(4, 3, 0);

    auto first = buffer.rent_for_writing();
    REQUIRE(first.has_value());
    auto second = buffer.rent_for_writing();
    REQUIRE(second.has_value());

    std::future<bool> parked = std::async(std::launch::async, [&buffer] {
        return buffer.rent_for_writing().has_value();
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // Nothing is ever read, so ending the stream is the only thing that can release the writer.
    fill_chunk(first->span(), 0, 4);
    std::move(*first).commit();
    fill_chunk(second->span(), 1, 4);
    std::move(*second).commit_final(4);

    const bool released = parked.wait_for(std::chrono::seconds(2)) == std::future_status::ready;

    // Freeing a slot releases the writer too, so a failure here reports instead of hanging.
    if (!released) {
        skip_chunk(buffer, 0);
    }

    CHECK(released);
    CHECK_FALSE(parked.get());
}
