#pragma once

#include <algorithm>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <span>
#include <tuple>
#include <utility>

#include <libassert/assert.hpp>

#include "containers/FixedArray.h"
#include "error/Error.h"
#include "error/Result.h"
#include "error/try.h"
#include "sink/Sink.h"

// Merges sorted streams into `output`.
template <typename T, typename Output, typename Compare>
    requires Sink<Output, T>
class MergingSink {
public:
    // What the output fails with.
    using Error = SinkError<Output, T>;

    MergingSink(const size_t source_count, Output output, Compare compare)
        : _sources(source_count),
          _output(std::forward<Output>(output)),
          _compare(std::forward<Compare>(compare)) {}

    alp::Result<void, Error> push(const size_t source, T value) {
        this->_sources[source].push(std::move(value));
        return this->drain();
    }

    alp::Result<void, Error> complete(const size_t source) {
        this->_sources[source].complete();
        ALP_TRY(this->drain());

        if (std::ranges::all_of(this->_sources, &Source::is_complete)) {
            return std::move(this->_output).finish();
        }
        return {};
    }

private:
    // One stream and its pending values.
    class Source {
    public:
        void push(T value) {
            DEBUG_ASSERT(!this->_is_complete);
            this->_pending.push_back(std::move(value));
        }

        void complete() {
            DEBUG_ASSERT(!this->_is_complete);
            this->_is_complete = true;
        }

        [[nodiscard]]
        bool is_complete() const {
            return this->_is_complete;
        }

    private:
        friend class MergingSink;

        [[nodiscard]]
        bool is_empty() const {
            return this->_pending.empty();
        }

        // Whether nothing is pending but more may come.
        [[nodiscard]]
        bool is_waiting() const {
            return this->is_empty() && !this->_is_complete;
        }

        [[nodiscard]]
        const T &front() const {
            return this->_pending.front();
        }

        T pop_front() {
            T value = std::move(this->_pending.front());
            this->_pending.pop_front();
            return value;
        }

        std::deque<T> _pending;
        bool _is_complete = false;
    };

    // Passes on every pending value no stream can still beat.
    alp::Result<void, Error> drain() {
        while (Source *source = this->find_next_source()) {
            ALP_TRY(this->_output.push(source->pop_front()));
        }
        return {};
    }

    // The stream with the smallest front, or null while one is waiting or none has a value.
    [[nodiscard]]
    Source *find_next_source() {
        Source *best = nullptr;
        for (Source &source : this->_sources) {
            if (source.is_waiting()) {
                return nullptr;
            }
            if (source.is_empty()) {
                continue;
            }
            if (best == nullptr || this->_compare(source.front(), best->front())) {
                best = &source;
            }
        }
        return best;
    }

    FixedArray<Source> _sources;
    Output _output;
    Compare _compare;
};

// One stream of a merging sink.
template <typename T, typename Output, typename Compare>
class MergingSinkInput {
    using Merge = MergingSink<T, Output, Compare>;

public:
    using Error = typename Merge::Error;

    MergingSinkInput(std::shared_ptr<Merge> merge, const size_t source)
        : _merge(std::move(merge)),
          _source(source) {}

    [[nodiscard]]
    alp::Result<void, Error> push(const std::span<const T> values) {
        for (const T &value : values) {
            ALP_TRY(this->_merge->push(this->_source, value));
        }
        return {};
    }

    [[nodiscard]]
    alp::Result<void, Error> finish() && {
        return this->_merge->complete(this->_source);
    }

private:
    std::shared_ptr<Merge> _merge;
    size_t _source;
};

// Merges `K` sorted streams into `output`, one input per stream.
template <size_t K, typename T, typename Output, typename Compare = std::less<>>
auto make_merging_sink(Output&& output, Compare && compare = {}) {
    using Sink = MergingSink<T, Output, Compare>;
    using Input = MergingSinkInput<T, Output, Compare>;

    const std::shared_ptr<Sink> merge = std::make_shared<Sink>(K, std::forward<Output>(output), std::forward<Compare>(compare));
    return [&merge]<size_t... Sources>(std::index_sequence<Sources...>) {
        return std::tuple{Input(merge, Sources)...};
    }(std::make_index_sequence<K>{});
}
