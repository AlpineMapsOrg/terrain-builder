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
#include <functional>
#include <limits>
#include <new>
#include <ranges>
#include <source_location>
#include <type_traits>
#include <utility>
#include <vector>

#include "common_error_codes.h"
#include "error/Error.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/limits.h"
#include "std_compat.h"

namespace io {

using AllocError = alp::Error<BufferLimitExceeded, OutOfMemory>;

// Calls `f`, and fails with OutOfMemory if it throws std::bad_alloc.
template <std::invocable F>
[[nodiscard]]
alp::Result<std::invoke_result_t<F>, OutOfMemory> catch_out_of_memory(F &&f, const std::source_location location = std::source_location::current()) {
    try {
        if constexpr (std::is_void_v<std::invoke_result_t<F>>) {
            std::invoke(std::forward<F>(f));
            return {};
        } else {
            return std::invoke(std::forward<F>(f));
        }
    } catch (const std::bad_alloc &) {
        return alp::make_error(OutOfMemory{}, location);
    }
}

// A vector of `size` value-initialized elements.
// Fails if the elements would take more than `max_bytes` or `max_buffer_size` bytes.
template <typename T>
[[nodiscard]]
alp::Result<std::vector<T>, AllocError> try_make_vector(const size_t size, const size_t max_bytes = max_buffer_size) {
    ALP_TRY(check_buffer_size<T>(size, max_bytes));
    return catch_out_of_memory([&] { return std::vector<T>(size); });
}

// A vector holding the elements of `range`.
// Fails if the elements would take more than `max_bytes` or `max_buffer_size` bytes.
template <std::ranges::sized_range Range>
[[nodiscard]]
alp::Result<std::vector<std::ranges::range_value_t<Range>>, AllocError> try_to_vector(Range &&range, const size_t max_bytes = max_buffer_size) {
    ALP_TRY(check_buffer_size<std::ranges::range_value_t<Range>>(std::ranges::size(range), max_bytes));
    return catch_out_of_memory([&] { return std::ranges::to<std::vector>(std::forward<Range>(range)); });
}

// Resizes `vector` to `size` elements, value-initializing the new ones; on failure `vector` is unchanged.
// Fails if the elements would take more than `max_bytes` or `max_buffer_size` bytes.
template <typename T>
[[nodiscard]]
alp::Result<void, AllocError> try_resize(std::vector<T> &vector, const size_t size, const size_t max_bytes = max_buffer_size) {
    ALP_TRY(check_buffer_size<T>(size, max_bytes));
    return catch_out_of_memory([&] { vector.resize(size); });
}

// Reserves room for at least `capacity` elements in `vector`; on failure `vector` is unchanged.
// Fails if that many elements would take more than `max_bytes` or `max_buffer_size` bytes.
template <typename T>
[[nodiscard]]
alp::Result<void, AllocError> try_reserve(std::vector<T> &vector, const size_t capacity, const size_t max_bytes = max_buffer_size) {
    ALP_TRY(check_buffer_size<T>(capacity, max_bytes));
    return catch_out_of_memory([&] { vector.reserve(capacity); });
}

// Appends the elements of `range` to `vector`, and leaves it unchanged if that fails.
// Fails if `vector` would then take more than `max_bytes` or `max_buffer_size` bytes.
template <typename T, std::ranges::sized_range Range>
[[nodiscard]]
alp::Result<void, AllocError> try_append_range(std::vector<T> &vector, Range &&range, const size_t max_bytes = max_buffer_size) {
    const size_t added_count = std::ranges::size(range);
    // A total this large has a byte size that size_t cannot hold.
    if (added_count > std::numeric_limits<size_t>::max() - vector.size()) {
        return alp::make_error(BufferLimitExceeded(std::min(max_bytes, max_buffer_size)));
    }
    ALP_TRY(check_buffer_size<T>(vector.size() + added_count, max_bytes));
    // Appending at the end has no effect if the allocation throws.
    return catch_out_of_memory([&] { std_compat::append_range(vector, std::forward<Range>(range)); });
}

} // namespace io
