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
#include <optional>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "error/Infallible.h"
#include "error/MessageCode.h"
#include "io/bytes.h"
#include "io/filesystem.h"
#include "stop.h"

using namespace tile_downloader;

namespace {

// Returns the file URL of `path`.
std::string make_file_url(const std::filesystem::path &path) {
    return "file://" + io::filesystem::canonicalize_best_effort(path).string();
}

struct SinkFailure : alp::MessageCode<"the sink failed"> {};

}

TEST_CASE("HttpClient::get keeps the body in memory", "[HttpClient]") {
    const TempFile source("atb-http-client.txt", "body");
    const std::string url = make_file_url(source.path());
    HttpClient client = REQUIRE_VALUE(HttpClient::make());

    const HttpResponse response = REQUIRE_VALUE(client.get(url));

    CHECK(response.body == io::bytes::vector{'b', 'o', 'd', 'y'});
}

TEST_CASE("HttpClient::get fails with TransportError for a missing file", "[HttpClient]") {
    const TempDirectory directory("atb-http-client-missing");
    const std::string url = make_file_url(directory.path() / "missing.txt");
    HttpClient client = REQUIRE_VALUE(HttpClient::make());

    const auto error = REQUIRE_ERROR(client.get(url));

    CHECK(error.holds<TransportError>());
}

TEST_CASE("HttpClient::get fails once the sink fails", "[HttpClient]") {
    const TempFile source("atb-http-client.txt", "response body");
    const std::string url = make_file_url(source.path());
    HttpClient client = REQUIRE_VALUE(HttpClient::make());
    const auto failing_sink = [](io::bytes::span) -> alp::Result<void, SinkFailure> { return alp::make_error(SinkFailure{}); };
    const auto keep_going = [](std::optional<double>) { return true; };

    const auto error = REQUIRE_ERROR(client.get(url, failing_sink, keep_going));

    CHECK(error.holds<SinkFailure>());
}

TEST_CASE("HttpClient::get fails with Stopped once the progress callback returns false", "[HttpClient]") {
    const TempFile source("atb-http-client.txt", "response body");
    const std::string url = make_file_url(source.path());
    HttpClient client = REQUIRE_VALUE(HttpClient::make());
    const auto ignoring_sink = [](io::bytes::span) -> alp::Result<void, alp::Infallible> { return {}; };
    const auto stop = [](std::optional<double>) { return false; };

    const auto error = REQUIRE_ERROR(client.get(url, ignoring_sink, stop));

    CHECK(error.holds<Stopped>());
}
