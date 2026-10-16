#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <optional>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

#include <libassert/assert.hpp>

#include "containers/spsc.h"
#include "range/Range.h"

namespace detail {

// Cuts the part of `span` that `range` covers.
template <typename T, size_t Extent>
std::span<T> slice(const std::span<T, Extent> span, const Range<size_t> &range) {
    DEBUG_ASSERT(range.is_in_bounds(span.size()));
    return span.subspan(range.start, range.size());
}

// One channel of a merge, with the values taken from it but not yet handed on.
template <typename T>
class MergeSource {
public:
    explicit MergeSource(spsc::Receiver<T> channel) : _channel(std::move(channel)) {}

    // Whether every value taken from the channel has been handed on.
    bool is_empty() const {
        return this->_ready.is_empty();
    }

    // Takes the next values from the channel, blocking until it has some or has ended.
    // Stays empty once the channel has ended.
    void refill_blocking() {
        this->_ready = Range<size_t>(0, this->_channel.pop_batch(this->_buffer));
    }

    // The next value to hand on. The source must not be empty.
    const T &front() const {
        return this->ready().front();
    }

    // Hands on the next value. The source must not be empty.
    T pop_front() {
        T value = std::move(this->ready().front());
        this->_ready.start++;
        return value;
    }

private:
    // How many values are taken from the channel at once.
    static constexpr size_t PREFETCH_COUNT = 64;

    // The values taken from the channel but not yet handed on.
    auto ready(this auto &&self) {
        return slice(std::span(self._buffer), self._ready);
    }

    spsc::Receiver<T> _channel;
    std::array<T, PREFETCH_COUNT> _buffer{};
    // Where the values not yet handed on lie in the buffer.
    Range<size_t> _ready{};
};

}

// Merges the sorted streams arriving on several channels into one sorted stream.
template <typename T, typename Compare>
class MergingReceiver {
public:
    MergingReceiver(std::vector<spsc::Receiver<T>> channels, Compare compare)
        : _compare(std::move(compare)) {
        this->_sources.reserve(channels.size());
        for (spsc::Receiver<T> &channel : channels) {
            this->_sources.emplace_back(std::move(channel));
        }
    }

    // Takes the smallest value still to come, or nothing once every source has ended.
    // Blocks until each source has either offered a value or ended.
    [[nodiscard]]
    std::optional<T> pop() {
        for (detail::MergeSource<T> &source : this->_sources) {
            if (source.is_empty()) {
                source.refill_blocking();
            }
        }

        auto pending = this->_sources | std::views::filter(std::not_fn(&detail::MergeSource<T>::is_empty));
        const auto best = std::ranges::min_element(pending, this->_compare, &detail::MergeSource<T>::front);
        if (best == pending.end()) {
            return std::nullopt;
        }
        return best->pop_front();
    }

private:
    std::vector<detail::MergeSource<T>> _sources;
    Compare _compare;
};

// Opens `source_count` channels, and the merge that consumes them.
template <typename T, typename Compare = std::less<>>
std::pair<std::vector<spsc::Sender<T>>, MergingReceiver<T, Compare>> make_merging_channels(
    const size_t source_count, const size_t capacity, Compare compare = {}) {
    std::vector<spsc::Sender<T>> senders;
    std::vector<spsc::Receiver<T>> receivers;
    senders.reserve(source_count);
    receivers.reserve(source_count);
    for (size_t source = 0; source < source_count; source++) {
        auto [sender, receiver] = spsc::make_channel<T>(capacity);
        senders.push_back(std::move(sender));
        receivers.push_back(std::move(receiver));
    }

    return std::pair{std::move(senders), MergingReceiver<T, Compare>(std::move(receivers), std::move(compare))};
}
