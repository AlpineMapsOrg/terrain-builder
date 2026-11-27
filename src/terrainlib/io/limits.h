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
#include <limits>
#include <optional>

#include "error/Result.h"
#include "error/utils.h"
#include "io/BufferLimitExceeded.h"

namespace io {

// The largest data buffer an io function creates or accepts.
inline constexpr size_t max_buffer_size = size_t{1} << 30;

// Computes the required buffer size for `count` elements of T, limited to `max_bytes` and `max_buffer_size`.
template <typename T>
[[nodiscard]]
alp::Result<size_t, BufferLimitExceeded> check_buffer_size(const size_t count, const size_t max_bytes) {
    const size_t limit = std::min(max_bytes, max_buffer_size);
    if (count <= limit / sizeof(T)) {
        return count * sizeof(T);
    }
    // A count this large has a byte size that size_t cannot hold.
    if (count > std::numeric_limits<size_t>::max() / sizeof(T)) {
        return alp::make_error(BufferLimitExceeded(limit));
    }
    return alp::make_error(BufferLimitExceeded(limit, count * sizeof(T)));
}

// Bytes that several buffers share, at most `max_buffer_size`.
class ByteBudget {
public:
    explicit ByteBudget(const size_t limit = max_buffer_size) : _limit(std::min(limit, max_buffer_size)) {}

    // Subtracts the required buffer size for `count` elements of T from the bytes left.
    template <typename T>
    [[nodiscard]] alp::Result<void, BufferLimitExceeded> try_claim(const size_t count) {
        auto size_result = check_buffer_size<T>(count, this->_limit - this->_claimed);
        if (!size_result.has_value()) {
            // check_buffer_size reports the bytes left as the limit, so the budget reports its own limit and the total instead.
            const std::optional<size_t> size_opt = size_result.error().code().size;
            // A total this large has a byte size that size_t cannot hold.
            if (!size_opt.has_value() || *size_opt > std::numeric_limits<size_t>::max() - this->_claimed) {
                return alp::make_error(BufferLimitExceeded(this->_limit));
            }
            return alp::make_error(BufferLimitExceeded(this->_limit, this->_claimed + *size_opt));
        }
        this->_claimed += *size_result;
        return {};
    }

private:
    size_t _limit;
    size_t _claimed = 0;
};

} // namespace io
