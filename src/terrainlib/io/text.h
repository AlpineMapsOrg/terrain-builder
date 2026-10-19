#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "error/Result.h"
#include "error/try.h"
#include "io/FileError.h"
#include "io/bytes.h"

namespace io {

// Writes `text` to the file at `path`, replacing its contents; with `make_dirs`, creates the missing parent directories first.
[[nodiscard]]
inline alp::Result<void, FileError> write_text_to_path(const std::string_view text, const std::filesystem::path &path, const bool make_dirs = true) {
    const std::vector<uint8_t> bytes(text.begin(), text.end());
    return write_bytes_to_path(bytes, path, make_dirs);
}

// The contents of the file at `path`, as text.
[[nodiscard]]
inline alp::Result<std::string, FileError> read_text_from_path(const std::filesystem::path &path) {
    ALP_TRY(const std::vector<uint8_t> bytes, read_bytes_from_path(path));
    return std::string(bytes.begin(), bytes.end());
}

} // namespace io
