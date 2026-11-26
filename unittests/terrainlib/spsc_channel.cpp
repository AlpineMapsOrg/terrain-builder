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

#include <array>
#include <atomic>
#include <chrono>
#include <expected>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include "containers/spsc.h"

namespace {

// Pops until the sender closes.
template <typename T>
std::vector<T> drain(spsc::Receiver<T> &receiver) {
    std::vector<T> values;
    while (std::optional<T> value = receiver.pop()) {
        values.push_back(*value);
    }
    return values;
}

std::vector<uint32_t> counting_up_to(const uint32_t count) {
    std::vector<uint32_t> values(count);
    std::iota(values.begin(), values.end(), 0);
    return values;
}

} // namespace

TEST_CASE("values come back in the order they were pushed", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    REQUIRE(sender.push(1));
    REQUIRE(sender.push(2));
    REQUIRE(sender.push(3));
    std::move(sender).close();

    CHECK(drain(receiver) == std::vector<uint32_t>{1, 2, 3});
}

TEST_CASE("a batch arrives as its individual elements", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    const std::array<uint32_t, 3> batch{1, 2, 3};
    REQUIRE(sender.push_batch(batch));
    std::move(sender).close();

    CHECK(drain(receiver) == std::vector<uint32_t>{1, 2, 3});
}

TEST_CASE("closing an empty channel ends the stream", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    std::move(sender).close();

    CHECK_FALSE(receiver.pop().has_value());
}

TEST_CASE("closing does not discard what was already queued", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    REQUIRE(sender.push(1));
    std::move(sender).close();

    REQUIRE(receiver.pop() == 1);
    CHECK_FALSE(receiver.pop().has_value());
}

TEST_CASE("the end of the stream is reported on every later pop", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    std::move(sender).close();

    CHECK_FALSE(receiver.pop().has_value());
    CHECK_FALSE(receiver.pop().has_value());
}

TEST_CASE("dropping the sender closes the channel", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    {
        spsc::Sender<uint32_t> owned = std::move(sender);
        REQUIRE(owned.push(1));
    }

    CHECK(drain(receiver) == std::vector<uint32_t>{1});
}

TEST_CASE("pushing to a dropped receiver reports the disconnection", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    {
        const spsc::Receiver<uint32_t> owned = std::move(receiver);
    }

    CHECK_FALSE(sender.push(7).has_value());
}

TEST_CASE("both endpoints can be moved", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    spsc::Sender<uint32_t> moved_sender = std::move(sender);
    spsc::Receiver<uint32_t> moved_receiver = std::move(receiver);

    REQUIRE(moved_sender.push(1));
    std::move(moved_sender).close();

    CHECK(drain(moved_receiver) == std::vector<uint32_t>{1});
}

TEST_CASE("size reports what is waiting to be popped", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    CHECK(receiver.size() == 0);

    REQUIRE(sender.push(1));
    REQUIRE(sender.push(2));
    CHECK(receiver.size() == 2);

    REQUIRE(receiver.pop() == 1);
    CHECK(receiver.size() == 1);
}

TEST_CASE("the ring wraps without losing or reordering values", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(4);

    std::vector<uint32_t> received;
    for (uint32_t i = 0; i < 20; i++) {
        REQUIRE(sender.push(i));
        const std::optional<uint32_t> value = receiver.pop();
        REQUIRE(value.has_value());
        received.push_back(*value);
    }

    CHECK(received == counting_up_to(20));
}

TEST_CASE("capacity is rounded up to a power of two", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(5);

    // A ring of five rounds to eight, so eight elements fit without a reader.
    const std::vector<uint32_t> batch = counting_up_to(8);
    REQUIRE(sender.push_batch(batch));
    CHECK(receiver.size() == 8);
}

TEST_CASE("push blocks while the ring is full", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(4);

    std::atomic<bool> finished = false;
    std::thread producer([&sender, &finished] {
        for (uint32_t i = 0; i < 16; i++) {
            if (!sender.push(i)) {
                return;
            }
        }
        finished = true;
        std::move(sender).close();
    });

    // The ring holds four, so the producer cannot have finished before anything is read.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    CHECK_FALSE(finished);

    const std::vector<uint32_t> received = drain(receiver);
    producer.join();

    CHECK(finished);
    CHECK(received == counting_up_to(16));
}

TEST_CASE("a batch larger than the ring is published in pieces", "[spsc_channel]") {
    constexpr uint32_t count = 1000;

    auto [sender, receiver] = spsc::make_channel<uint32_t>(16);
    const std::vector<uint32_t> batch = counting_up_to(count);

    std::thread producer([&sender, &batch] {
        if (sender.push_batch(batch)) {
            std::move(sender).close();
        }
    });

    const std::vector<uint32_t> received = drain(receiver);
    producer.join();

    CHECK(received == batch);
}

TEST_CASE("closing the receiver disconnects the channel", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    std::move(receiver).close();

    CHECK_FALSE(sender.push(1).has_value());
}

TEST_CASE("closing the receiver wakes a blocked sender", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(4);

    std::atomic<bool> disconnected = false;
    std::thread producer([&sender, &disconnected] {
        for (uint32_t i = 0; i < 1000; i++) {
            if (!sender.push(i).has_value()) {
                disconnected = true;
                return;
            }
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::move(receiver).close();
    producer.join();

    CHECK(disconnected);
}

TEST_CASE("pop waits for a value instead of reporting the end", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    bool sent = false;
    std::thread producer([&sender, &sent] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        sent = sender.push(42).has_value();
        std::move(sender).close();
    });

    CHECK(drain(receiver) == std::vector<uint32_t>{42});
    producer.join();

    CHECK(sent);
}

TEST_CASE("end to end, a producer and a consumer pass a long stream", "[spsc_channel]") {
    constexpr uint32_t count = 200000;

    auto [sender, receiver] = spsc::make_channel<uint32_t>(64);

    // Catch2 assertions are not thread safe, so the producer only records what it saw.
    bool all_sent = true;
    std::thread producer([&sender, &all_sent] {
        for (uint32_t i = 0; i < count; i++) {
            all_sent = all_sent && sender.push(i).has_value();
        }
        std::move(sender).close();
    });

    const std::vector<uint32_t> received = drain(receiver);
    producer.join();

    CHECK(all_sent);
    CHECK(received == counting_up_to(count));
}

TEST_CASE("end to end, batched pushes survive many wraps", "[spsc_channel]") {
    constexpr uint32_t count = 200000;
    constexpr uint32_t batch_size = 97;

    auto [sender, receiver] = spsc::make_channel<uint32_t>(64);
    const std::vector<uint32_t> values = counting_up_to(count);

    bool all_sent = true;
    std::thread producer([&] {
        for (uint32_t sent = 0; sent < count; sent += batch_size) {
            const uint32_t size = std::min(batch_size, count - sent);
            const std::span<const uint32_t> batch(values.data() + sent, size);
            all_sent = all_sent && sender.push_batch(batch).has_value();
        }
        std::move(sender).close();
    });

    const std::vector<uint32_t> received = drain(receiver);
    producer.join();

    CHECK(all_sent);
    CHECK(received == values);
}

TEST_CASE("a batch pop takes everything that is queued", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    const std::array<uint32_t, 3> batch{1, 2, 3};
    REQUIRE(sender.push_batch(batch));

    std::array<uint32_t, 8> out{};
    REQUIRE(receiver.pop_batch(out) == 3);
    CHECK(std::vector<uint32_t>(out.begin(), out.begin() + 3) == std::vector<uint32_t>{1, 2, 3});
}

TEST_CASE("a batch pop takes no more than the span holds", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    const std::vector<uint32_t> batch = counting_up_to(6);
    REQUIRE(sender.push_batch(batch));

    std::array<uint32_t, 2> out{};
    REQUIRE(receiver.pop_batch(out) == 2);
    CHECK(out[0] == 0);
    CHECK(out[1] == 1);
    CHECK(receiver.size() == 4);
}

TEST_CASE("a batch pop reports the end of the stream as nothing taken", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(8);

    std::move(sender).close();

    std::array<uint32_t, 4> out{};
    CHECK(receiver.pop_batch(out) == 0);
}

TEST_CASE("a batch pop wraps around the end of the ring", "[spsc_channel]") {
    auto [sender, receiver] = spsc::make_channel<uint32_t>(4);

    const std::array<uint32_t, 3> first{1, 2, 3};
    REQUIRE(sender.push_batch(first));
    std::array<uint32_t, 3> drained{};
    REQUIRE(receiver.pop_batch(drained) == 3);

    const std::array<uint32_t, 4> second{4, 5, 6, 7};
    REQUIRE(sender.push_batch(second));

    std::array<uint32_t, 4> out{};
    REQUIRE(receiver.pop_batch(out) == 4);
    CHECK(std::vector<uint32_t>(out.begin(), out.end()) == std::vector<uint32_t>{4, 5, 6, 7});
}

TEST_CASE("move assigning a sender ends the channel it left", "[spsc_channel]") {
    auto [first_sender, first_receiver] = spsc::make_channel<uint32_t>(8);
    auto [second_sender, second_receiver] = spsc::make_channel<uint32_t>(8);

    REQUIRE(first_sender.push(1));
    first_sender = std::move(second_sender);

    CHECK(drain(first_receiver) == std::vector<uint32_t>{1});
}

TEST_CASE("move assigning a receiver disconnects the channel it left", "[spsc_channel]") {
    auto [first_sender, first_receiver] = spsc::make_channel<uint32_t>(8);
    auto [second_sender, second_receiver] = spsc::make_channel<uint32_t>(8);

    first_receiver = std::move(second_receiver);

    CHECK_FALSE(first_sender.push(1));
}
