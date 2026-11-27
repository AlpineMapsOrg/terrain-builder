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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <tuple>

#include <fmt/format.h>

#include "containers/UniqueResource.h"
#include "io/bytes.h"
#include "io/filesystem.h"
#include "io/filesystem_utils.h"
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

// A name that no other TempDirectory or TempFile uses.
inline std::string make_unique_name(const std::string_view label) {
    static std::atomic<uint64_t> counter = 0;
    const uint64_t index = counter++;
    const auto timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const std::string label_suffix = label.empty() ? std::string() : fmt::format("-{}", label);
    return fmt::format("atb-test{}-{}-{}", label_suffix, timestamp, index);
}

// A file name that no other TempFile uses, with the extension of `file_name`.
inline std::string make_unique_file_name(const std::filesystem::path &file_name) {
    return make_unique_name(file_name.stem().string()) + file_name.extension().string();
}
} // namespace detail

// A fresh directory under the unit test output tree, removed again when the test ends.
class TempDirectory {
public:
    explicit TempDirectory(const std::string_view label = {})
        : _path(std::filesystem::path(ALP_TEST_OUTPUT_DIR) / detail::make_unique_name(label)) {
        REQUIRE_VALUE(io::filesystem::create_directories(this->_path.get()));
    }

    const std::filesystem::path &path() const {
        return this->_path.get();
    }

private:
    UniqueResource<std::filesystem::path, detail::RemoveDirectory> _path;
};

// A path for a file under the unit test output tree with the extension of `name`, removed again when the test ends.
class TempFile {
public:
    explicit TempFile(const std::string_view name)
        : _path(std::filesystem::path(ALP_TEST_OUTPUT_DIR) / detail::make_unique_file_name(name)) {
        REQUIRE_VALUE(io::filesystem::create_parent_directories(this->_path.get()));
    }

    TempFile(const std::string_view name, const std::string_view text)
        : TempFile(name) {
        REQUIRE_VALUE(io::text::write(text, this->path()));
    }

    TempFile(const std::string_view name, const std::span<const uint8_t> bytes)
        : TempFile(name) {
        REQUIRE_VALUE(io::bytes::write(bytes, this->path()));
    }

    const std::filesystem::path &path() const {
        return this->_path.get();
    }

private:
    UniqueResource<std::filesystem::path, detail::RemoveFile> _path;
};
