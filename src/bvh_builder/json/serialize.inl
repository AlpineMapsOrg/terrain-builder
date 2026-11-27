#pragma once

#include <cstdint>
#include <vector>

#include <fmt/format.h>
#include <glaze/json.hpp>

#include "error/try.h"
#include "error/utils.h"
#include "io/bytes.h"
#include "type_utils.h"

namespace json {

namespace detail {

// Settings that glz::opts does not carry have to be declared by a struct derived from it.
struct WriteOptions : glz::opts {
    bool new_lines_in_arrays = false;
};

struct ReadOptions : glz::opts {
    bool error_on_missing_array_elements = true;
};

inline constexpr WriteOptions WRITE_OPTIONS{{.prettify = true}};
inline constexpr ReadOptions READ_OPTIONS{{.null_terminated = false, .error_on_missing_keys = true}};

// Writes `value` as JSON into a new buffer of type `Buffer`.
template <typename Buffer, typename T>
alp::Result<Buffer, JsonError> write_to_buffer(const T &value) {
    Buffer buffer;
    const glz::error_ctx error = glz::write<WRITE_OPTIONS>(value, buffer);
    if (error) {
        return alp::make_error(JsonError(error.ec, glz::format_error(error, buffer)), fmt::format("encoding {}", type_name<T>()));
    }
    return buffer;
}

// Reads the value that `buffer` holds as JSON.
template <typename T, typename Buffer>
alp::Result<T, JsonError> read_from_buffer(const Buffer &buffer) {
    T value{};
    const glz::error_ctx error = glz::read<READ_OPTIONS>(value, buffer);
    if (error) {
        return alp::make_error(JsonError(error.ec, glz::format_error(error, buffer)), fmt::format("decoding {}", type_name<T>()));
    }
    return value;
}

}

template <typename T>
alp::Result<std::string, JsonError> write_to_text(const T &value) {
    return detail::write_to_buffer<std::string>(value);
}

template <typename T>
alp::Result<T, JsonError> read_from_text(const std::string_view text) {
    return detail::read_from_buffer<T>(text);
}

template <typename T>
alp::Result<void, io::FileError, JsonError> write_to_path(const T &value, const std::filesystem::path &path, const bool make_dirs) {
    const auto context = alp::make_context("writing {} to {}", type_name<T>(), path);
    ALP_TRY_CONTEXT(const std::vector<uint8_t> &bytes, detail::write_to_buffer<std::vector<uint8_t>>(value), context);
    ALP_TRY_CONTEXT(io::write_bytes_to_path(bytes, path, make_dirs), context);
    return {};
}

template <typename T>
alp::Result<T, io::FileError, JsonError> read_from_path(const std::filesystem::path &path) {
    const auto context = alp::make_context("reading {} from {}", type_name<T>(), path);
    ALP_TRY_CONTEXT(const std::vector<uint8_t> &bytes, io::read_bytes_from_path(path), context);
    ALP_TRY_CONTEXT(T value, detail::read_from_buffer<T>(bytes), context);
    return value;
}

}
