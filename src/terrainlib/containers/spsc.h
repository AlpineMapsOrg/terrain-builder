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

#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>

#include <libassert/assert.hpp>

#include "containers/FixedArray.h"
#include "containers/UniqueResource.h"

namespace spsc {

template <typename T>
class Sender;

template <typename T>
class Receiver;

template <typename T>
[[nodiscard]] std::pair<Sender<T>, Receiver<T>> make_channel(const size_t capacity);

// Reports to a push that the receiving end is gone.
struct Disconnected {};

namespace detail {
inline constexpr size_t cache_line = 64;

// How far one end of the channel has got, together with the flags that end can raise.
struct Cursor {
    static constexpr uint64_t ended = uint64_t{1} << 63;
    static constexpr uint64_t count_mask = ended - 1;

    uint64_t raw = 0;

    // Counting never reaches the flags, so an end can publish with a plain fetch_add.
    [[nodiscard]]
    uint64_t count() const {
        return this->raw & count_mask;
    }

    // Raised by the sender once the stream is closed, and by the receiver once it is gone.
    [[nodiscard]]
    bool has_ended() const {
        return (this->raw & ended) != 0;
    }
};

template <typename T>
struct State {
    static_assert(std::is_trivially_copyable_v<T>);
    static_assert(std::is_default_constructible_v<T>);

    explicit State(const size_t requested)
        : capacity(std::bit_ceil(std::max<size_t>(requested, 1))),
          buffer(this->capacity) {
    }

    // Where a position in the stream lands in the ring.
    [[nodiscard]]
    size_t get_slot_index(const uint64_t position) const {
        return position & (this->capacity - 1);
    }

    // Copies the values into the ring, splitting them at the wrap.
    void write(const uint64_t position, const std::span<const T> values) {
        const size_t offset = this->get_slot_index(position);
        const size_t before_wrap = std::min(values.size(), this->capacity - offset);

        std::copy_n(values.begin(), before_wrap, this->buffer.data() + offset);
        std::copy_n(values.begin() + before_wrap, values.size() - before_wrap, this->buffer.data());
    }

    // Copies as many values out of the ring as out holds, splitting them at the wrap.
    void read(const uint64_t position, const std::span<T> out) const {
        const size_t offset = this->get_slot_index(position);
        const size_t before_wrap = std::min(out.size(), this->capacity - offset);

        std::copy_n(this->buffer.data() + offset, before_wrap, out.begin());
        std::copy_n(this->buffer.data(), out.size() - before_wrap, out.begin() + before_wrap);
    }

    [[nodiscard]]
    Cursor load_head(const std::memory_order order) const {
        return Cursor{this->head.load(order)};
    }

    [[nodiscard]]
    Cursor load_tail(const std::memory_order order) const {
        return Cursor{this->tail.load(order)};
    }

    // Blocks while the ring is full, and returns how far the receiver had got.
    Cursor wait_for_space(const uint64_t position) {
        Cursor observed = this->load_head(std::memory_order_acquire);
        while (position - observed.count() == this->capacity) {
            if (observed.has_ended()) {
                break;
            }

            this->head.wait(observed.raw, std::memory_order_relaxed);
            observed = this->load_head(std::memory_order_acquire);
        }
        return observed;
    }

    // Blocks while the ring is empty, and returns how far the sender had got.
    Cursor wait_for_data(const uint64_t position) {
        Cursor observed = this->load_tail(std::memory_order_acquire);
        while (observed.count() == position) {
            if (observed.has_ended()) {
                break;
            }

            this->tail.wait(observed.raw, std::memory_order_relaxed);
            observed = this->load_tail(std::memory_order_acquire);
        }
        return observed;
    }

    const size_t capacity;
    FixedArray<T> buffer;

    // Kept apart so that publishing does not invalidate the line the other end reads from.
    alignas(cache_line) std::atomic<uint64_t> tail = 0;
    alignas(cache_line) std::atomic<uint64_t> head = 0;
};

// Tells a blocked receiver that nothing more will be published.
template <typename T>
struct EndStream {
    void operator()(std::shared_ptr<State<T>> &state) const noexcept {
        state->tail.fetch_or(Cursor::ended, std::memory_order_release);
        state->tail.notify_all();
    }
};

// Tells a blocked sender that nothing more will be consumed.
template <typename T>
struct EndSink {
    void operator()(std::shared_ptr<State<T>> &state) const noexcept {
        state->head.fetch_or(Cursor::ended, std::memory_order_release);
        state->head.notify_all();
    }
};

} // namespace detail

// The producing end. Closing it, by hand or by dropping it, ends the stream.
template <typename T>
class Sender {
public:
    Sender(Sender &&) noexcept = default;
    Sender &operator=(Sender &&) noexcept = default;

    // Blocks while the ring is full. A batch larger than the ring is published in pieces.
    [[nodiscard]]
    std::expected<void, Disconnected> push_batch(const std::span<const T> values) {
        DEBUG_ASSERT(this->_state.is_held(), "the sender was already closed");
        detail::State<T> &state = *this->_state.get();

        std::span<const T> remaining = values;
        uint64_t position = state.load_tail(std::memory_order_relaxed).count();
        while (!remaining.empty()) {
            const detail::Cursor head = state.wait_for_space(position);
            if (head.has_ended()) {
                return std::unexpected(Disconnected{});
            }

            const size_t free = state.capacity - (position - head.count());
            const std::span<const T> chunk = remaining.first(std::min(remaining.size(), free));
            state.write(position, chunk);

            position += chunk.size();
            state.tail.fetch_add(chunk.size(), std::memory_order_release);
            state.tail.notify_one();

            remaining = remaining.subspan(chunk.size());
        }
        return {};
    }

    // Publishes one value immediately, so a loop of these pays a wakeup per element.
    [[nodiscard]]
    std::expected<void, Disconnected> push(const T value) {
        return this->push_batch(std::span<const T>(&value, 1));
    }

    // Ends the stream; dropping the sender does the same.
    void close() && {
        this->_state.reset();
    }

private:
    explicit Sender(std::shared_ptr<detail::State<T>> state)
        : _state(std::move(state)) {
    }

    friend std::pair<Sender<T>, Receiver<T>> make_channel<T>(size_t);

    UniqueResource<std::shared_ptr<detail::State<T>>, detail::EndStream<T>> _state;
};

// The consuming end. Dropping it disconnects the channel, so that later pushes fail.
template <typename T>
class Receiver {
public:
    Receiver(Receiver &&) noexcept = default;
    Receiver &operator=(Receiver &&) noexcept = default;

    // Blocks until at least one value is queued, then takes up to as many as fit in out.
    // Returns how many it took, or zero once the sender has closed and the ring is empty.
    [[nodiscard]]
    size_t pop_batch(const std::span<T> out) {
        DEBUG_ASSERT(this->_state.is_held(), "the receiver was already dropped");
        DEBUG_ASSERT(!out.empty(), "an empty out cannot tell a value apart from the end");
        detail::State<T> &state = *this->_state.get();

        const uint64_t position = state.load_head(std::memory_order_relaxed).count();
        const detail::Cursor tail = state.wait_for_data(position);

        const size_t queued = tail.count() - position;
        const size_t count = std::min(queued, out.size());
        if (count == 0) {
            return 0;
        }

        state.read(position, out.first(count));
        state.head.fetch_add(count, std::memory_order_release);
        state.head.notify_one();
        return count;
    }

    // Blocks until a value is queued. Returns nothing once the sender has closed and the
    // ring is empty.
    [[nodiscard]]
    std::optional<T> pop() {
        T value{};
        if (this->pop_batch(std::span<T>(&value, 1)) == 0) {
            return std::nullopt;
        }
        return value;
    }

    // Disconnects the channel; dropping the receiver does the same.
    void close() && {
        this->_state.reset();
    }

    [[nodiscard]]
    size_t size() const {
        const detail::State<T> &state = *this->_state.get();
        const detail::Cursor tail = state.load_tail(std::memory_order_acquire);
        const detail::Cursor head = state.load_head(std::memory_order_relaxed);
        return tail.count() - head.count();
    }

private:
    explicit Receiver(std::shared_ptr<detail::State<T>> state)
        : _state(std::move(state)) {
    }

    friend std::pair<Sender<T>, Receiver<T>> make_channel<T>(size_t);

    UniqueResource<std::shared_ptr<detail::State<T>>, detail::EndSink<T>> _state;
};

// Creates a channel whose ring holds capacity elements, rounded up to a power of two.
template <typename T>
[[nodiscard]]
std::pair<Sender<T>, Receiver<T>> make_channel(const size_t capacity) {
    auto state = std::make_shared<detail::State<T>>(capacity);
    return {Sender<T>(state), Receiver<T>(std::move(state))};
}

} // namespace spsc
