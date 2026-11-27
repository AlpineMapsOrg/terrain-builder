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
#include <cstddef>
#include <expected>
#include <optional>
#include <span>
#include <utility>

#include "containers/ChunkedRingBuffer.h"
#include "containers/UniqueResource.h"
#include "error/try.h"

// Fills a ring's chunks from a stream of elements, ending the stream on finish().
// A writer that goes away before that aborts the stream.
template <typename T>
class ChunkedRingWriter {
public:
    using Ring = ChunkedRingBuffer<T>;

    explicit ChunkedRingWriter(Ring &ring) : _stream(PendingStream(ring)) {}

    // Reports how the stream ended if it did, leaving the rest of `elements` unwritten.
    [[nodiscard]]
    std::expected<void, typename Ring::StreamEnd> write(const std::span<const T> elements) {
        std::span<const T> remaining = elements;
        while (!remaining.empty()) {
            if (this->_stream->unwritten.empty()) {
                ALP_TRY(this->advance_chunk());
            }

            const size_t taken = std::min(this->_stream->unwritten.size(), remaining.size());
            std::copy_n(remaining.begin(), taken, this->_stream->unwritten.begin());
            this->_stream->unwritten = this->_stream->unwritten.subspan(taken);
            remaining = remaining.subspan(taken);
        }
        return {};
    }

    // Publishes the elements in hand and ends the stream.
    void finish() && {
        PendingStream stream = this->_stream.take();
        if (!stream.chunk.has_value()) {
            stream.ring->complete();
            return;
        }

        const std::span<T> chunk = stream.chunk->span();
        std::move(stream.chunk).value().commit_final(chunk.size() - stream.unwritten.size());
    }

private:
    // The chunk in hand and the room left in it, until the stream is finished.
    struct PendingStream {
        explicit PendingStream(Ring &ring) noexcept : ring(&ring) {}

        Ring *ring;
        std::optional<typename Ring::WriteLease> chunk;
        std::span<T> unwritten;
    };

    struct AbortStream {
        void operator()(PendingStream &stream) const noexcept {
            // Dropping the lease ends the stream, but only a rented chunk has one to drop.
            stream.chunk.reset();
            stream.ring->abort();
        }
    };

    // Publishes the chunk in hand and rents the next one.
    [[nodiscard]]
    std::expected<void, typename Ring::StreamEnd> advance_chunk() {
        if (this->_stream->chunk.has_value()) {
            std::move(this->_stream->chunk).value().commit();
            this->_stream->chunk.reset();
        }

        ALP_TRY(this->_stream->chunk, this->_stream->ring->rent_for_writing());
        this->_stream->unwritten = this->_stream->chunk->span();
        return {};
    }

    UniqueResource<PendingStream, AbortStream> _stream;
};
