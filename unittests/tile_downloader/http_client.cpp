/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include "HttpClient.h"

#include <filesystem>
#include <stdexcept>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "../temp_directory.h"
#include "io/filesystem.h"

namespace {

// Returns the file URL of `path`.
std::string make_file_url(const std::filesystem::path &path) {
    return "file://" + io::filesystem::canonicalize_best_effort(path).string();
}

class CallbackError : public std::runtime_error {
public:
    CallbackError()
        : std::runtime_error("callback failed") {}
};

}

TEST_CASE("http client propagates response writer exceptions") {
    const TempFile source("atb-http-client.txt", "response body");
    HttpClient client([](std::vector<char> &, const char *, size_t) {
        throw CallbackError();
    });

    CHECK_THROWS_AS(client.get(make_file_url(source.path())), CallbackError);
}

TEST_CASE("http client propagates progress callback exceptions") {
    const TempFile source("atb-http-client.txt", "response body");
    HttpClient client;

    CHECK_THROWS_AS(
        client.get(make_file_url(source.path()), [](double) {
            throw CallbackError();
        }),
        CallbackError);
}
