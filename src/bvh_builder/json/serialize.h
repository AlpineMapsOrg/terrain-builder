#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include "error/Result.h"
#include "io/FileError.h"
#include "io/bytes.h"

#include "json/JsonError.h"

namespace json {

// Writes `value` as JSON text.
template <typename T>
[[nodiscard]]
alp::Result<std::string, JsonError> write_to_text(const T &value);

// Reads the value that `text` holds as JSON.
// Fails on unknown keys, missing keys and incomplete vectors.
template <typename T>
[[nodiscard]]
alp::Result<T, JsonError> read_from_text(std::string_view text);

// Writes `value` as JSON to the file at `path`.
// Creates the parent directories first if `make_dirs` is set.
template <typename T>
[[nodiscard]]
alp::Result<void, io::FileError, JsonError> write_to_path(const T &value, const std::filesystem::path &path, bool make_dirs = true);

// Reads the value that the file at `path` holds as JSON.
template <typename T>
[[nodiscard]]
alp::Result<T, io::bytes::ReadError, JsonError> read_from_path(const std::filesystem::path &path);

}

#include "json/serialize.inl"
