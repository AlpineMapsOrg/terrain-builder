#pragma once

#include <algorithm>
#include <cstddef>
#include <span>

#include <libassert/assert.hpp>

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
