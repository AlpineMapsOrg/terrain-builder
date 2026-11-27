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

#include <expected>
#include <new>
#include <system_error>

#include <fmt/format.h>
#include <libassert/assert.hpp>
#include <zpp_bits.h>

#include "error/try.h"
#include "error/utils.h"
#include "fmt_impls.h"
#include "io/ZppBitsError.h"
#include "io/limits.h"
#include "type_name.h"

namespace io::serde {

namespace detail {

// The serialization error for a code zpp_bits reported, created from that code.
inline SerializeError to_serialize_error(const ZppBitsError error) {
    switch (error.code()) {
    case std::errc::no_buffer_space:
    case std::errc::message_size:
        return alp::make_error(BufferLimitExceeded(max_buffer_size), error);
    case std::errc::invalid_argument:
        return alp::make_error(Unserializable{}, error);
    // zpp_bits documents these codes for fixed-size buffers, for reading and for remote procedure calls.
    case std::errc::result_out_of_range:
    case std::errc::value_too_large:
    case std::errc::bad_message:
    case std::errc::not_supported:
    case std::errc::protocol_error:
        PANIC("zpp_bits reported a code serialize does not expect", fmt::format("{}", error));
    // A code zpp_bits does not document comes from the value's own serialization functions.
    default:
        return alp::make_error(Unserializable{}, error);
    }
}

// The deserialization error for a code zpp_bits reported while decoding with the allocation limit `limit`, created from that code.
inline DeserializeError to_deserialize_error(const ZppBitsError error, const size_t limit) {
    switch (error.code()) {
    case std::errc::result_out_of_range:
        return alp::make_error(Truncated{}, error);
    case std::errc::message_size:
        return alp::make_error(BufferLimitExceeded(limit), error);
    case std::errc::value_too_large:
    case std::errc::bad_message:
        return alp::make_error(CorruptData{}, error);
    // zpp_bits documents these codes for writing and for remote procedure calls.
    case std::errc::invalid_argument:
    case std::errc::no_buffer_space:
    case std::errc::not_supported:
    case std::errc::protocol_error:
        PANIC("zpp_bits reported a code deserialize does not expect", fmt::format("{}", error));
    // A code zpp_bits does not document comes from the value's own deserialization functions.
    default:
        return alp::make_error(CorruptData{}, error);
    }
}

} // namespace detail

template <typename T>
alp::Result<bytes::vector, SerializeError> serialize(const T &value) {
    const auto context = alp::make_context("encoding {}", type_name<T>());
    bytes::vector data;
    zpp::bits::out out(data, zpp::bits::endian::little{}, zpp::bits::alloc_limit<max_buffer_size>{});
    try {
        const auto result = out(value);
        if (zpp::bits::failure(result)) {
            return detail::to_serialize_error(ZppBitsError(result)).context(context);
        }
    } catch (const std::bad_alloc &) {
        return alp::make_error(OutOfMemory{}, context);
    }
    return data;
}

template <typename T, size_t MaxSize>
alp::Result<T, DeserializeError> deserialize(const bytes::span bytes) {
    static_assert(MaxSize <= max_buffer_size, "the limit must not exceed max_buffer_size");
    const auto context = alp::make_context("decoding {}", type_name<T>());
    zpp::bits::in in(bytes, zpp::bits::endian::little{}, zpp::bits::alloc_limit<MaxSize>{});
    try {
        T value = zpp::bits::access::make<T>();
        const auto result = in(value);
        if (zpp::bits::failure(result)) {
            return detail::to_deserialize_error(ZppBitsError(result), MaxSize).context(context);
        }
        if (in.position() != bytes.size()) {
            return alp::make_error(TrailingData{}, context);
        }
        return value;
    } catch (const std::bad_alloc &) {
        return alp::make_error(OutOfMemory{}, context);
    }
}

template <typename T, size_t MaxSize>
alp::Result<Prefix<T>, DeserializePrefixError> deserialize_prefix(const bytes::span bytes) {
    auto prefix_result = serde::deserialize<Prefix<T>, MaxSize>(bytes);
    if (!prefix_result.has_value()) {
        DeserializeError error = std::move(prefix_result).error();
        std::expected<alp::Error<TrailingData>, DeserializePrefixError> split_result = std::move(error).split<TrailingData>();
        ASSERT(!split_result.has_value(), "decoding a prefix reported data after the value");
        return std::move(split_result).error();
    }
    return std::move(prefix_result).value();
}

template <typename T>
alp::Result<T, ReadError> read(const std::filesystem::path &path) {
    const auto context = alp::make_context("reading {} from {}", type_name<T>(), path);
    ALP_TRY_CONTEXT(const bytes::vector &bytes, bytes::read(path), context);
    ALP_TRY_CONTEXT(T value, serde::deserialize<T>(bytes), context);
    return value;
}

template <typename T>
alp::Result<void, WriteError> write(const T &value, const std::filesystem::path &path, const WriteOptions options) {
    const auto context = alp::make_context("writing {} to {}", type_name<T>(), path);
    ALP_TRY_CONTEXT(const bytes::vector &bytes, serde::serialize(value), context);
    ALP_TRY_CONTEXT(bytes::write(bytes, path, options), context);
    return {};
}

} // namespace io::serde
