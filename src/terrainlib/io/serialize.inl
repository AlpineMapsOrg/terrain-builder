#pragma once

#include <fmt/format.h>
#include <zpp_bits.h>

#include "error/try.h"
#include "error/utils.h"
#include "io/SerdeError.h"
#include "io/bytes.h"
#include "type_utils.h"

namespace io {

template <typename T>
alp::Result<std::vector<uint8_t>, SerdeError> write_to_bytes(const T &value) {
    std::vector<uint8_t> data;
    zpp::bits::out out(data);
    const auto result = out(value);
    if (zpp::bits::failure(result)) {
        return alp::make_error(SerdeError(result), fmt::format("encoding {}", type_name<T>()));
    }
    return data;
}

template <typename T>
alp::Result<T, SerdeError> read_from_bytes(const std::span<const uint8_t> bytes) {
    zpp::bits::in in(bytes);
    T value = zpp::bits::access::make<T>();
    const auto result = in(value);
    if (zpp::bits::failure(result)) {
        return alp::make_error(SerdeError(result), fmt::format("decoding {}", type_name<T>()));
    }
    return value;
}

template <typename T>
Result<T> read_from_path(const std::filesystem::path &path) {
    const auto context = alp::make_context("reading {} from {}", type_name<T>(), path);
    ALP_TRY_CONTEXT(const std::vector<uint8_t> &bytes, read_bytes_from_path(path), context);
    ALP_TRY_CONTEXT(T value, read_from_bytes<T>(bytes), context);
    return value;
}

template <typename T>
Result<void> write_to_path(const T &value, const std::filesystem::path &path, bool make_dirs) {
    const auto context = alp::make_context("writing {} to {}", type_name<T>(), path);
    ALP_TRY_CONTEXT(const std::vector<uint8_t> &bytes, write_to_bytes(value), context);
    ALP_TRY_CONTEXT(write_bytes_to_path(bytes, path, make_dirs), context);
    return {};
}

} // namespace io
