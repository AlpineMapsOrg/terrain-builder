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

#include <cstddef>
#include <expected>
#include <span>
#include <atomic>
#include <algorithm>
#include <limits>
#include <mutex>
#include <optional>
#include <utility>

#include <libassert/assert.hpp>

#include "containers/FixedArray.h"
#include "containers/UniqueResource.h"
#include "log.h"


// A run of elements with some of its neighbours kept on either side.
template <typename T>
class PaddedSpan {
public:
    PaddedSpan() = default;

    PaddedSpan(const std::span<T> span, const size_t left_padding, const size_t right_padding) noexcept
        : _span(span),
          _left_padding(left_padding),
          _right_padding(right_padding) {
        DEBUG_ASSERT(span.size() >= left_padding + right_padding);
    }

    // The run and both of its paddings.
    [[nodiscard]]
    std::span<T> span() const noexcept {
        return this->_span;
    }

    // A sub-range of the whole run, offset from the start of the left padding.
    [[nodiscard]]
    PaddedSpan slice(const size_t offset, const size_t count) const noexcept {
        DEBUG_ASSERT(offset + count <= this->_span.size());

        const size_t cut_right = this->_span.size() - offset - count;
        return {
            this->_span.subspan(offset, count),
            this->_left_padding - std::min(offset, this->_left_padding),
            this->_right_padding - std::min(cut_right, this->_right_padding),
        };
    }

    // The run on its own.
    [[nodiscard]]
    std::span<T> data() const noexcept {
        return this->_span.subspan(this->_left_padding, this->size());
    }

    [[nodiscard]]
    size_t size() const noexcept {
        return this->_span.size() - this->_left_padding - this->_right_padding;
    }

    [[nodiscard]]
    size_t left_padding() const noexcept {
        return this->_left_padding;
    }

    [[nodiscard]]
    size_t right_padding() const noexcept {
        return this->_right_padding;
    }

    [[nodiscard]]
    PaddedSpan trim_padding_left() const noexcept {
        return this->slice(this->_left_padding, this->_span.size() - this->_left_padding);
    }

    [[nodiscard]]
    PaddedSpan trim_padding_right() const noexcept {
        return this->slice(0, this->_span.size() - this->_right_padding);
    }

    [[nodiscard]]
    PaddedSpan trim_padding() const noexcept {
        return this->trim_padding_left().trim_padding_right();
    }

private:
    std::span<T> _span;
    size_t _left_padding = 0;
    size_t _right_padding = 0;
};

// A ring of equally sized chunks, each of which can be taken with a halo of its neighbours.
template <typename T>
class ChunkRing {
public:
    ChunkRing(const size_t chunk_size, const size_t num_chunks, const size_t padding) :
        _chunk_size(chunk_size),
        _num_chunks(num_chunks),
        _padding(padding),
        _data(chunk_size * num_chunks + padding * 2) {
        ASSERT(num_chunks >= 3);
        ASSERT(padding <= chunk_size);
    }

    [[nodiscard]]
    size_t chunk_size() const noexcept {
        return this->_chunk_size;
    }

    [[nodiscard]]
    size_t num_chunks() const noexcept {
        return this->_num_chunks;
    }

    [[nodiscard]]
    size_t padding() const noexcept {
        return this->_padding;
    }

    [[nodiscard]]
    size_t get_slot_index(const size_t chunk_index) const noexcept {
        return chunk_index % this->_num_chunks;
    }

    // The chunk without any halo.
    [[nodiscard]]
    std::span<T> chunk(const size_t chunk_index) noexcept {
        return {this->get_chunk_data(chunk_index), this->_chunk_size};
    }

    // The chunk padded with its neighbours on either side.
    [[nodiscard]]
    auto chunk_with_halo(this auto &&self, const size_t chunk_index) noexcept {
        auto *begin = self.get_chunk_data(chunk_index) - self._padding;
        return PaddedSpan{std::span{begin, self._padding + self._chunk_size + self._padding}, self._padding, self._padding};
    }

    // Gives the chunks at the two ends of the ring the neighbour they cannot reach directly.
    void copy_seam_halo(const size_t chunk_index) noexcept {
        if (chunk_index == 0 || this->get_slot_index(chunk_index) != 0) {
            return;
        }

        const size_t ring_end = this->_padding + this->_num_chunks * this->_chunk_size;
        T *front_pad = this->_data.data();
        T *back_pad = this->_data.data() + ring_end;
        const T *predecessor_tail = back_pad - this->_padding;
        const T *first_chunk_head = front_pad + this->_padding;

        std::copy_n(predecessor_tail, this->_padding, front_pad);
        std::copy_n(first_chunk_head, this->_padding, back_pad);
    }

private:
    auto get_chunk_data(this auto &&self, const size_t chunk_index) noexcept {
        return self._data.data() + self._padding + self.get_slot_index(chunk_index) * self._chunk_size;
    }

    const size_t _chunk_size;
    const size_t _num_chunks;
    const size_t _padding;

    // [_padding][_num_chunks * _chunk_size][_padding]
    FixedArray<T> _data;
};

template <typename T>
class ChunkedRingBuffer {
    enum class SlotState : uint8_t {
        Empty,
        Written,
        Abandoned,
        Done,
    };

    struct RentedChunk {
        ChunkedRingBuffer *buffer = nullptr;
        size_t index = 0;
    };

    struct RetireChunk {
        void operator()(RentedChunk &chunk) const noexcept {
            chunk.buffer->release_chunk(chunk.index);
        }
    };

    struct AbandonChunk {
        void operator()(RentedChunk &chunk) const noexcept {
            LOG_WARN("write lease was neither committed nor abandoned, aborting the stream");
            chunk.buffer->abandon_chunk(chunk.index);
        }
    };

public:
    enum class StreamState : uint8_t {
        Running,
        Completed,
        Aborted,
    };

    // How a stream finished.
    enum class StreamEnd : uint8_t {
        Completed,
        Aborted,
    };

    // Why a rent was refused.
    enum class RentError : uint8_t {
        WouldBlock,
        Completed,
        Aborted,
    };

    ChunkedRingBuffer(const size_t chunk_size, const size_t num_chunks, const size_t padding) :
        _ring(chunk_size, num_chunks, padding),
        _slot_state(num_chunks, SlotState::Empty) {
    }

    ChunkedRingBuffer(const ChunkedRingBuffer &) = delete;
    ChunkedRingBuffer &operator=(const ChunkedRingBuffer &) = delete;

    class WriteLease {
    public:
        [[nodiscard]]
        std::span<T> span() const noexcept {
            return this->_data;
        }

        [[nodiscard]]
        size_t chunk_index() const noexcept {
            return this->_chunk->index;
        }

        void commit() && {
            const RentedChunk chunk = this->_chunk.take();
            chunk.buffer->commit_chunk(chunk.index, this->_data.size(), false);
        }

        // Ends the stream; `written` may be shorter than the chunk.
        void commit_final(const size_t written) && {
            DEBUG_ASSERT(written > 0 && written <= this->_data.size());

            const RentedChunk chunk = this->_chunk.take();
            chunk.buffer->commit_chunk(chunk.index, written, true);
        }

        // Aborts the stream, discarding the chunk.
        void abandon() && {
            const RentedChunk chunk = this->_chunk.take();
            chunk.buffer->abandon_chunk(chunk.index);
        }

    private:
        friend class ChunkedRingBuffer;

        WriteLease(ChunkedRingBuffer &buffer, const size_t chunk_index, const std::span<T> data) noexcept
            : _chunk(RentedChunk{&buffer, chunk_index}, AbandonChunk{}),
              _data(data) {}

        UniqueResource<RentedChunk, AbandonChunk> _chunk;
        std::span<T> _data;
    };

    class ReadLease {
    public:
        // The chunk together with its halo, readable until this lease goes away.
        [[nodiscard]]
        PaddedSpan<const T> view() const noexcept {
            return this->_view;
        }

        [[nodiscard]]
        size_t chunk_index() const noexcept {
            return this->_chunk->index;
        }

    private:
        friend class ChunkedRingBuffer;

        ReadLease(ChunkedRingBuffer &buffer, const size_t chunk_index, const PaddedSpan<const T> view) noexcept
            : _chunk(RentedChunk{&buffer, chunk_index}, RetireChunk{}),
              _view(view) {}

        UniqueResource<RentedChunk, RetireChunk> _chunk;
        PaddedSpan<const T> _view;
    };

    [[nodiscard]]
    StreamState stream_state() const noexcept {
        return this->_stream_state.load(std::memory_order_acquire);
    }

    // Blocks until the ring has room. Reports how the stream ended once it has.
    [[nodiscard]]
    std::expected<WriteLease, StreamEnd> rent_for_writing() {
        while (true) {
            const uint32_t seen = this->_writable_epoch.load(std::memory_order_acquire);
            std::expected<WriteLease, RentError> result = this->try_rent_for_writing();
            if (result.has_value()) {
                return std::move(result).value();
            }
            if (result.error() != RentError::WouldBlock) {
                return std::unexpected(get_stream_end(result.error()));
            }
            this->_writable_epoch.wait(seen, std::memory_order_acquire);
        }
    }

    // Blocks until the next chunk can be read. Reports how the stream ended once it has.
    [[nodiscard]]
    std::expected<ReadLease, StreamEnd> rent_for_reading() {
        while (true) {
            const uint32_t seen = this->_readable_epoch.load(std::memory_order_acquire);
            std::expected<ReadLease, RentError> result = this->try_rent_for_reading();
            if (result.has_value()) {
                return std::move(result).value();
            }
            if (result.error() != RentError::WouldBlock) {
                return std::unexpected(get_stream_end(result.error()));
            }
            this->_readable_epoch.wait(seen, std::memory_order_acquire);
        }
    }

    [[nodiscard]]
    std::expected<WriteLease, RentError> try_rent_for_writing() {
        size_t chunk_index = this->_next_write_chunk.load(std::memory_order_relaxed);
        while (true) {
            const StreamState state = this->_stream_state.load(std::memory_order_acquire);
            if (state != StreamState::Running) {
                return std::unexpected(get_rent_error(state));
            }
            if (!this->is_slot_free(chunk_index)) {
                return std::unexpected(RentError::WouldBlock);
            }
            if (this->_next_write_chunk.compare_exchange_weak(chunk_index, chunk_index + 1, std::memory_order_relaxed)) {
                return this->make_write_lease(chunk_index);
            }
        }
    }

    // Drops the rest of the stream, leaving both sides nothing to wait for.
    void abort() {
        std::unique_lock lock(this->_advance_mutex);
        this->end_stream(StreamState::Aborted);
        lock.unlock();

        this->wake_after_write(0, true);
    }

    // Ends the stream on the chunks committed so far.
    void complete() {
        std::unique_lock lock(this->_advance_mutex);
        DEBUG_ASSERT(this->_next_write_chunk.load(std::memory_order_relaxed) == this->_next_written_chunk.load(std::memory_order_relaxed), "a chunk is still rented for writing");
        this->end_stream(StreamState::Completed);
        lock.unlock();

        this->wake_after_write(0, true);
    }

    [[nodiscard]]
    std::expected<ReadLease, RentError> try_rent_for_reading() {
        size_t chunk_index = this->_next_read_chunk.load(std::memory_order_relaxed);
        while (true) {
            const size_t written = this->_next_written_chunk.load(std::memory_order_acquire);
            const StreamState state = this->_stream_state.load(std::memory_order_acquire);
            const bool ended = state != StreamState::Running;
            if (ended && chunk_index >= this->_final_chunk_count) {
                return std::unexpected(get_rent_error(state));
            }

            // A chunk can be read once the one after it exists, or once the stream ends on it.
            const bool has_halo = written >= chunk_index + 2;
            const bool is_last = ended && chunk_index + 1 == this->_final_chunk_count && written >= chunk_index + 1;
            if (!has_halo && !is_last) {
                return std::unexpected(RentError::WouldBlock);
            }
            if (this->_next_read_chunk.compare_exchange_weak(chunk_index, chunk_index + 1, std::memory_order_relaxed)) {
                return this->make_read_lease(chunk_index);
            }
        }
    }

private:
    // How a rent reports a stream that is no longer running.
    static RentError get_rent_error(const StreamState state) noexcept {
        DEBUG_ASSERT(state != StreamState::Running);
        return state == StreamState::Aborted ? RentError::Aborted : RentError::Completed;
    }

    // How the stream ended, for a rent that was refused because it had.
    static StreamEnd get_stream_end(const RentError error) noexcept {
        DEBUG_ASSERT(error != RentError::WouldBlock);
        return error == RentError::Aborted ? StreamEnd::Aborted : StreamEnd::Completed;
    }

    // Ends the stream after `chunk_count` chunks, of which the last holds `final_size` entries.
    void end_stream(const StreamState state, const size_t chunk_count, const size_t final_size) {
        DEBUG_ASSERT(state != StreamState::Running);
        if (this->_stream_state.load(std::memory_order_relaxed) != StreamState::Running) {
            return;
        }

        this->_final_chunk_size = final_size;
        this->_final_chunk_count = chunk_count;
        this->_stream_state.store(state, std::memory_order_release);
    }

    // Ends the stream on the chunks committed so far.
    void end_stream(const StreamState state) {
        const size_t chunk_count = this->_next_written_chunk.load(std::memory_order_relaxed);
        this->end_stream(state, chunk_count, this->_ring.chunk_size());
    }

    // A writer may take the slot once no reader can still reach into it from either side.
    bool is_slot_free(const size_t chunk_index) const noexcept {
        const size_t retired = this->_retired_read_chunk.load(std::memory_order_acquire);
        return chunk_index < retired + this->_ring.num_chunks() - 1;
    }

    // How many elements the stream is known to hold. Only the last chunk is ever a short one.
    size_t get_stream_length() const noexcept {
        if (this->_stream_state.load(std::memory_order_acquire) == StreamState::Running) {
            return this->_next_written_chunk.load(std::memory_order_acquire) * this->_ring.chunk_size();
        }
        if (this->_final_chunk_count == 0) {
            // empty stream
            return 0;
        }
        return (this->_final_chunk_count - 1) * this->_ring.chunk_size() + this->_final_chunk_size;
    }

    PaddedSpan<const T> get_read_view(const size_t chunk_index) const noexcept {
        PaddedSpan<const T> view = this->_ring.chunk_with_halo(chunk_index);
        if (chunk_index == 0) {
            view = view.trim_padding_left();
        }

        // The last two chunks reach past the end of the stream, so their view is cut short.
        const size_t view_end = (chunk_index + 1) * this->_ring.chunk_size() + this->_ring.padding();
        const size_t length = this->get_stream_length();
        if (view_end <= length) {
            return view;
        }
        return view.slice(0, view.span().size() - (view_end - length));
    }

    WriteLease make_write_lease(const size_t chunk_index) noexcept {
        return WriteLease{*this, chunk_index, this->_ring.chunk(chunk_index)};
    }

    ReadLease make_read_lease(const size_t chunk_index) noexcept {
        return ReadLease{*this, chunk_index, this->get_read_view(chunk_index)};
    }

    void commit_chunk(const size_t chunk_index, const size_t written, const bool is_final) {
        std::unique_lock lock(this->_advance_mutex);

        if (is_final) {
            this->end_stream(StreamState::Completed, chunk_index + 1, written);
        }

        this->_slot_state[this->_ring.get_slot_index(chunk_index)] = SlotState::Written;
        const size_t published = this->advance_written();
        const bool ended = this->_stream_state.load(std::memory_order_relaxed) != StreamState::Running;
        lock.unlock();

        this->wake_after_write(published, ended);
    }

    void abandon_chunk(const size_t chunk_index) noexcept {
        std::unique_lock lock(this->_advance_mutex);

        this->_slot_state[this->_ring.get_slot_index(chunk_index)] = SlotState::Abandoned;
        const size_t published = this->advance_written();
        const bool ended = this->_stream_state.load(std::memory_order_relaxed) != StreamState::Running;
        lock.unlock();

        this->wake_after_write(published, ended);
    }

    void release_chunk(const size_t chunk_index) {
        std::unique_lock lock(this->_advance_mutex);

        this->_slot_state[this->_ring.get_slot_index(chunk_index)] = SlotState::Done;
        const size_t retired = this->advance_retired();
        lock.unlock();

        if (retired == 0) {
            return;
        }

        // Every retired chunk frees one slot, which lets exactly one writer through.
        this->_writable_epoch.fetch_add(1, std::memory_order_release);
        for (size_t i = 0; i < retired; i++) {
            this->_writable_epoch.notify_one();
        }
    }

    // Stops at the first chunk nobody rented: beyond that a slot still holds the previous lap's state.
    size_t advance_written() {
        const size_t rented = this->_next_write_chunk.load(std::memory_order_relaxed);
        size_t chunk_index = this->_next_written_chunk.load(std::memory_order_relaxed);
        while (chunk_index < rented) {
            const SlotState state = this->_slot_state[this->_ring.get_slot_index(chunk_index)];
            if (state == SlotState::Abandoned) {
                this->end_stream(StreamState::Aborted, chunk_index, this->_ring.chunk_size());
                break;
            }
            if (state != SlotState::Written) {
                break;
            }

            this->_ring.copy_seam_halo(chunk_index);
            chunk_index++;
        }
        const size_t before = this->_next_written_chunk.load(std::memory_order_relaxed);
        this->_next_written_chunk.store(chunk_index, std::memory_order_release);
        return chunk_index - before;
    }

    size_t advance_retired() {
        const size_t rented = this->_next_read_chunk.load(std::memory_order_relaxed);
        size_t chunk_index = this->_retired_read_chunk.load(std::memory_order_relaxed);
        while (chunk_index < rented && this->_slot_state[this->_ring.get_slot_index(chunk_index)] == SlotState::Done) {
            this->_slot_state[this->_ring.get_slot_index(chunk_index)] = SlotState::Empty;
            chunk_index++;
        }
        const size_t before = this->_retired_read_chunk.load(std::memory_order_relaxed);
        this->_retired_read_chunk.store(chunk_index, std::memory_order_release);
        return chunk_index - before;
    }

    // Every published chunk releases one reader, while the end of the stream releases
    // everyone, since nobody parked on either side can be given anything after it.
    void wake_after_write(const size_t published, const bool ended) noexcept {
        if (published == 0 && !ended) {
            return;
        }

        this->_readable_epoch.fetch_add(1, std::memory_order_release);
        if (!ended) {
            for (size_t i = 0; i < published; i++) {
                this->_readable_epoch.notify_one();
            }
            return;
        }

        this->_readable_epoch.notify_all();
        this->_writable_epoch.fetch_add(1, std::memory_order_release);
        this->_writable_epoch.notify_all();
    }

    ChunkRing<T> _ring;

    // Guards the slot states and both counters below that only ever move in order
    std::mutex _advance_mutex;
    // [slot]: state of the chunk currently held by this slot
    FixedArray<SlotState> _slot_state;

    // The next chunk to rent for writing.
    std::atomic<size_t> _next_write_chunk = 0;
    // The next chunk that was written.
    std::atomic<size_t> _next_written_chunk = 0;
    // Publishes the two fields below, which are only valid once it is not Running.
    std::atomic<StreamState> _stream_state = StreamState::Running;
    // The number of chunks in the stream.
    size_t _final_chunk_count = 0;
    // Number of valid entries in the final chunk.
    size_t _final_chunk_size = 0;

    // The next chunk to rent for reading
    std::atomic<size_t> _next_read_chunk = 0;
    // The oldest chunk that is still being read
    std::atomic<size_t> _retired_read_chunk = 0;

    // One park queue per side, so that a wakeup always reaches a thread that can use it.
    std::atomic<uint32_t> _readable_epoch = 0;
    std::atomic<uint32_t> _writable_epoch = 0;
};
