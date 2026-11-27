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
#include <concepts>
#include <cstddef>
#include <memory>
#include <ranges>
#include <utility>

template <typename T>
class FixedArray {
public:
    explicit FixedArray(const size_t size)
        : _size(size),
          _data(size == 0 ? nullptr : std::make_unique<T[]>(size)) {}

    FixedArray(const size_t size, const T &value) : FixedArray(size) {
        std::ranges::fill(this->span(), value);
    }

    FixedArray(const FixedArray &other) : FixedArray(other._size) {
        std::ranges::copy(other.span(), this->begin());
    }

    FixedArray &operator=(const FixedArray &other) {
        if (this == &other) {
            return *this;
        }

        FixedArray copy(other);
        *this = std::move(copy);
        return *this;
    }

    FixedArray(FixedArray &&) noexcept = default;
    FixedArray &operator=(FixedArray &&) noexcept = default;

    [[nodiscard]]
    size_t size() const noexcept {
        return this->_size;
    }

    [[nodiscard]]
    bool empty() const noexcept {
        return this->_size == 0;
    }

    T& operator[](size_t index) noexcept {
        return this->_data[index];
    }

    const T& operator[](size_t index) const noexcept {
        return this->_data[index];
    }

    T* data() noexcept {
        return this->_data.get();
    }

    const T* data() const noexcept {
        return this->_data.get();
    }

    T* begin() noexcept {
        return this->_data.get();
    }

    const T* begin() const noexcept {
        return this->_data.get();
    }

    T* end() noexcept {
        return this->_data.get() + this->_size;
    }

    const T* end() const noexcept {
        return this->_data.get() + this->_size;
    }

    std::span<T> span() noexcept {
        return {this->_data.get(), this->_size};
    }

    std::span<const T> span() const noexcept {
        return {this->_data.get(), this->_size};
    }

private:
    const size_t _size;
    std::unique_ptr<T[]> _data;
};
