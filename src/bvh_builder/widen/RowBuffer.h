#pragma once

#include <cstdint>
#include <limits>
#include <span>
#include <vector>

#include <libassert/assert.hpp>
#include <zpp_bits.h>

#include "std_compat.h"

namespace widen {

// A row per node, stored back to back with a seperate length buffer.
template <typename T, typename Length>
class RowBuffer {
public:
    class Iterator {
    public:
        [[nodiscard]] std::span<const T> operator*() const {
            const std::span<const T> data(this->_buffer->_rows);
            return data.subspan(this->_offset, this->_buffer->_lengths[this->_row]);
        }

        Iterator &operator++() {
            this->_offset += this->_buffer->_lengths[this->_row];
            this->_row++;
            return *this;
        }

        Iterator &operator--() {
            this->_row--;
            this->_offset -= this->_buffer->_lengths[this->_row];
            return *this;
        }

        [[nodiscard]] bool operator==(const Iterator &other) const {
            return this->_row == other._row;
        }

    private:
        friend class RowBuffer;

        Iterator(const RowBuffer &buffer, const size_t row, const size_t offset) : _buffer(&buffer), _row(row), _offset(offset) {}

        const RowBuffer *_buffer;
        size_t _row;
        size_t _offset;
    };

    [[nodiscard]] size_t size() const {
        return this->_lengths.size();
    }

    [[nodiscard]] std::span<const Length> lengths() const {
        return this->_lengths;
    }

    [[nodiscard]] std::span<const T> values() const {
        return this->_rows;
    }

    [[nodiscard]] Iterator begin() const {
        return Iterator(*this, 0, 0);
    }

    [[nodiscard]] Iterator end() const {
        return Iterator(*this, this->size(), this->_rows.size());
    }

    void push_row(const std::span<const T> row) {
        ASSERT(row.size() <= std::numeric_limits<Length>::max());

        std_compat::append_range(this->_rows, row);
        this->_lengths.push_back(static_cast<Length>(row.size()));
    }

private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<2>;

    std::vector<Length> _lengths;
    std::vector<T> _rows;
};

}