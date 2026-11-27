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

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <system_error>

#include "io/FileError.h"

TEST_CASE("A file error compares equal to the condition its code matches") {
    const io::FileError error(std::make_error_code(std::errc::no_such_file_or_directory), "some/file");

    CHECK(error == std::errc::no_such_file_or_directory);
    CHECK(error != std::errc::permission_denied);
}

TEST_CASE("A file error compares equal to one on the same path whose code means the same", "[io][FileError]") {
    // Code 2 in the system category is ENOENT on POSIX and ERROR_FILE_NOT_FOUND on Windows.
    const io::FileError system(std::error_code(2, std::system_category()), "some/file");

    CHECK(system == io::FileError(std::make_error_code(std::errc::no_such_file_or_directory), "some/file"));
    CHECK(system != io::FileError(std::make_error_code(std::errc::permission_denied), "some/file"));
    CHECK(system != io::FileError(std::make_error_code(std::errc::no_such_file_or_directory), "other/file"));
}
