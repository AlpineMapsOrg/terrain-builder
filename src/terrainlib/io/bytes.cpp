#include <fstream>
#include <system_error>

#include <fmt/format.h>

#include "error/try.h"
#include "error/utils.h"
#include "io/bytes.h"
#include "io/filesystem.h"
#include "log.h"

namespace io {

alp::Result<void, FileError> write_bytes_to_path(const std::span<const uint8_t> bytes, const std::filesystem::path &path, bool make_dirs) {
    LOG_TRACE("Writing bytes to path {}", path);
    const auto open_context = alp::make_context("opening {} for writing", path);
    const auto write_context = alp::make_context("writing bytes to {}", path);

    if (make_dirs) {
        ALP_TRY(filesystem::create_parent_directories(path));
    }

    // A stream reports no reason for a failure, so we return a general io error.
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        return alp::make_unexpected(FileError(std::make_error_code(std::errc::io_error), path), open_context);
    }
    file.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    if (!file.good()) {
        return alp::make_unexpected(FileError(std::make_error_code(std::errc::io_error), path), write_context);
    }

    return {};
}

alp::Result<std::vector<uint8_t>, FileError> read_bytes_from_path(const std::filesystem::path &path) {
    LOG_TRACE("Reading bytes from path {}", path);
    const auto open_context = alp::make_context("opening {} for reading", path);
    const auto read_context = alp::make_context("reading bytes from {}", path);

    std::error_code code;
    const std::uintmax_t size = std::filesystem::file_size(path, code);
    if (code) {
        return alp::make_unexpected(FileError(code, path), open_context);
    }

    // A stream reports no reason for a failure, so we return a general io error.
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        return alp::make_unexpected(FileError(std::make_error_code(std::errc::io_error), path), open_context);
    }
    std::vector<uint8_t> buffer(static_cast<size_t>(size));
    file.read(reinterpret_cast<char *>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
    if (!file.good()) {
        return alp::make_unexpected(FileError(std::make_error_code(std::errc::io_error), path), read_context);
    }

    return buffer;
}

} // namespace io
