#pragma once

#include <cstdint>
#include <filesystem>
#include <span>
#include <string_view>
#include <tuple>

#include "containers/UniqueResource.h"
#include "io/bytes.h"
#include "io/filesystem.h"
#include "io/text.h"
#include "result_helpers.h"

namespace detail {
// Removes the directory at a path with everything in it.
struct RemoveDirectory {
    void operator()(const std::filesystem::path &path) const noexcept {
        std::ignore = io::filesystem::remove_all(path);
    }
};

// Removes the file at a path.
struct RemoveFile {
    void operator()(const std::filesystem::path &path) const noexcept {
        std::ignore = io::filesystem::remove(path);
    }
};
} // namespace detail

// A fresh directory under the unit test output tree, removed again when the test ends.
class TempDirectory {
public:
    explicit TempDirectory(const std::string_view name)
        : _path(std::filesystem::path(ALP_TEST_OUTPUT_DIR) / name) {
        REQUIRE_VALUE(io::filesystem::remove_all(this->_path.get()));
        REQUIRE_VALUE(io::filesystem::create_directories(this->_path.get()));
    }

    const std::filesystem::path &path() const {
        return this->_path.get();
    }

private:
    UniqueResource<std::filesystem::path, detail::RemoveDirectory> _path;
};

// A path for a file under the unit test output tree, removed again when the test ends.
class TempFile {
public:
    explicit TempFile(const std::string_view name)
        : _path(std::filesystem::path(ALP_TEST_OUTPUT_DIR) / name) {
        REQUIRE_VALUE(io::filesystem::remove(this->_path.get()));
        REQUIRE_VALUE(io::filesystem::create_parent_directories(this->_path.get()));
    }

    TempFile(const std::string_view name, const std::string_view text)
        : TempFile(name) {
        REQUIRE_VALUE(io::write_text_to_path(text, this->path()));
    }

    TempFile(const std::string_view name, const std::span<const uint8_t> bytes)
        : TempFile(name) {
        REQUIRE_VALUE(io::write_bytes_to_path(bytes, this->path()));
    }

    const std::filesystem::path &path() const {
        return this->_path.get();
    }

private:
    UniqueResource<std::filesystem::path, detail::RemoveFile> _path;
};
