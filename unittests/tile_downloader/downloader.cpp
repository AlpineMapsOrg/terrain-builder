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

#include "Downloader.h"

#include <filesystem>
#include <string>
#include <string_view>
#include <utility>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "io/bytes.h"
#include "io/filesystem.h"
#include "io/text.h"

using namespace tile_downloader;

namespace {

class TemporaryPyramid {
public:
    explicit TemporaryPyramid(std::string_view name)
        : _directory(name) {}

    [[nodiscard]] const std::filesystem::path &path() const {
        return this->_directory.path();
    }

    [[nodiscard]] std::filesystem::path tile_path(const radix::tile::Id &tile) const {
        return make_google_tile_path(this->path(), tile, ".jpeg");
    }

    void create_pending(const radix::tile::Id &tile) const {
        const std::filesystem::path path = this->tile_path(tile);
        const io::bytes::vector data{'t', 'i', 'l', 'e'};
        REQUIRE_VALUE(details::write_file_children_pending(path, data));
    }

    void create_complete(const radix::tile::Id &tile) const {
        create_pending(tile);
        REQUIRE_VALUE(details::mark_tile_children_complete(this->tile_path(tile)));
    }

private:
    TempDirectory _directory;
};

const TileUrlBuilder missing_file_url({
    "file:///definitely-missing-atb-tile/{zoom}/{x}/{y}.jpeg",
    TileYDirection::Down
});

}

TEST_CASE("write_file_children_pending writes the tile as pending", "[Downloader]") {
    const TempDirectory directory("atb-write-file-success");
    const std::filesystem::path path = directory.path() / "tile.bin";
    const std::filesystem::path pending_path = details::children_pending_tile_path(path);
    const std::string expected = "tile";
    const io::bytes::vector data(expected.begin(), expected.end());

    REQUIRE_VALUE(details::write_file_children_pending(path, data));

    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(path)));
    CHECK(REQUIRE_VALUE(io::text::read(pending_path)) == expected);
}

TEST_CASE("write_file_children_pending names the pending path when the write fails", "[Downloader]") {
    const TempDirectory directory("atb-write-file-failure");
    const std::filesystem::path path = directory.path() / "tile";
    const std::filesystem::path pending_path = details::children_pending_tile_path(path);
    const io::bytes::vector data{'t', 'i', 'l', 'e'};
    REQUIRE(REQUIRE_VALUE(io::filesystem::create_directory(pending_path)));

    const auto error = REQUIRE_ERROR(details::write_file_children_pending(path, data));

    CHECK(error.code().path() == pending_path);
}

TEST_CASE("mark_tile_children_complete keeps the tile pending when the final path is taken", "[Downloader]") {
    const TempDirectory directory("atb-write-file-promotion-failure");
    const std::filesystem::path path = directory.path() / "tile";
    const std::filesystem::path pending_path = details::children_pending_tile_path(path);
    const io::bytes::vector data{'t', 'i', 'l', 'e'};
    REQUIRE(REQUIRE_VALUE(io::filesystem::create_directory(path)));
    REQUIRE_VALUE(details::write_file_children_pending(path, data));

    const auto error = REQUIRE_ERROR(details::mark_tile_children_complete(path));

    CHECK(error.code().path() == path);
    CHECK(REQUIRE_VALUE(io::filesystem::is_directory(path)));
    CHECK(REQUIRE_VALUE(io::filesystem::exists(pending_path)));
}

TEST_CASE("tile downloader promotes parents after completed children", "[Downloader]") {
    const TemporaryPyramid pyramid("atb-downloader-complete-pyramid");
    const radix::tile::Id root{0, {0, 0}};
    const auto children = root.children();

    pyramid.create_pending(root);
    for (const auto &child : children) {
        pyramid.create_pending(child);
    }

    HttpClient http = REQUIRE_VALUE(HttpClient::make());
    Downloader downloader(missing_file_url, std::move(http), pyramid.path(), 1u, root.zoom_level);
    REQUIRE_VALUE(downloader.download_recursive(root));

    REQUIRE(REQUIRE_VALUE(io::filesystem::exists(pyramid.tile_path(root))));
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(details::children_pending_tile_path(pyramid.tile_path(root)))));

    for (const auto &child : children) {
        CHECK(REQUIRE_VALUE(io::filesystem::exists(pyramid.tile_path(child))));
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(details::children_pending_tile_path(pyramid.tile_path(child)))));
    }

    REQUIRE(REQUIRE_VALUE(io::filesystem::remove(pyramid.tile_path(children.front()))));
    HttpClient resumed_http = REQUIRE_VALUE(HttpClient::make());
    Downloader resumed_downloader(missing_file_url, std::move(resumed_http), pyramid.path(), 1u, root.zoom_level);
    REQUIRE_VALUE(resumed_downloader.download_recursive(root));
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(pyramid.tile_path(children.front()))));
}

TEST_CASE("tile downloader leaves ancestors pending after a child failure", "[Downloader]") {
    const TemporaryPyramid pyramid("atb-downloader-failed-pyramid");
    const radix::tile::Id root{0, {0, 0}};
    const auto children = root.children();

    pyramid.create_pending(root);
    for (size_t i = 1; i < children.size(); ++i) {
        pyramid.create_complete(children[i]);
    }

    HttpClient http = REQUIRE_VALUE(HttpClient::make());
    Downloader downloader(missing_file_url, std::move(http), pyramid.path(), 1u, root.zoom_level);
    CHECK(REQUIRE_ERROR(downloader.download_recursive(root)).holds_code(IncompleteDownload{1}));

    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(pyramid.tile_path(root))));
    CHECK(REQUIRE_VALUE(io::filesystem::exists(details::children_pending_tile_path(pyramid.tile_path(root)))));
    for (size_t i = 1; i < children.size(); ++i) {
        CHECK(REQUIRE_VALUE(io::filesystem::exists(pyramid.tile_path(children[i]))));
    }
}

TEST_CASE("tile downloader still tries the siblings of a tile that fails", "[Downloader]") {
    const TemporaryPyramid pyramid("atb-downloader-failed-siblings");
    const radix::tile::Id root{0, {0, 0}};
    pyramid.create_pending(root);

    HttpClient http = REQUIRE_VALUE(HttpClient::make());
    Downloader downloader(missing_file_url, std::move(http), pyramid.path(), 1u, root.zoom_level);

    CHECK(REQUIRE_ERROR(downloader.download_recursive(root)).holds_code(IncompleteDownload{4}));
    CHECK(REQUIRE_VALUE(io::filesystem::exists(details::children_pending_tile_path(pyramid.tile_path(root)))));
}
