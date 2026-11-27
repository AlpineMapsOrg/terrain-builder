/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Martin Braunsperger
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>

#include <fmt/format.h>
#include <radix/tile.h>

#include "HttpClient.h"
#include "TileLogger.h"
#include "TileUrlBuilder.h"
#include "error/Result.h"
#include "error/log.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/FileError.h"
#include "io/WriteMode.h"
#include "io/bytes.h"
#include "io/filesystem.h"
#include "tile_path.h"

namespace tile_downloader {

namespace details {

// The path the tile at `path` has while its children are not complete.
[[nodiscard]] inline std::filesystem::path children_pending_tile_path(const std::filesystem::path &path) {
    auto pending_path = path;
    pending_path += ".children-pending";
    return pending_path;
}

// Writes `data` as the tile at `path` whose children are not complete.
[[nodiscard]] inline alp::Result<void, io::FileError> write_file_children_pending(const std::filesystem::path &path, const io::bytes::span data) {
    // Replace keeps the old or the new bytes even after a crash, so a pending tile is never truncated.
    const io::bytes::WriteOptions options{.mode = io::WriteMode::Replace};
    const std::filesystem::path pending_path = children_pending_tile_path(path);
    ALP_TRY_CONTEXT(io::bytes::write(data, pending_path, options), "writing a pending tile");
    return {};
}

// Marks the tile at `path` as complete with all its children.
[[nodiscard]] inline alp::Result<void, io::FileError> mark_tile_children_complete(const std::filesystem::path &path) {
    const std::filesystem::path pending_path = children_pending_tile_path(path);
    ALP_TRY_CONTEXT(io::filesystem::rename(pending_path, path), "completing a tile");
    return {};
}

} // namespace details

// The response is not an image.
struct BadContentType {
    std::string content_type;

    bool operator==(const BadContentType &) const = default;
};

inline std::string format_as(const BadContentType &error) {
    return fmt::format("the response has content type \"{}\" instead of an image", error.content_type);
}

// What fetching a tile from the server can fail with.
using FetchTileError = alp::Error<GetError, BadContentType>;

// Some tiles could not be downloaded, so their ancestors are still pending.
struct IncompleteDownload {
    size_t failed_tile_count;

    bool operator==(const IncompleteDownload &) const = default;
};

inline std::string format_as(const IncompleteDownload &error) {
    return fmt::format("{} tiles could not be downloaded", error.failed_tile_count);
}

// What downloading tiles can fail with.
using DownloadError = alp::Error<io::FileError, IncompleteDownload>;

// How downloading a tile ended, when it did not fail.
enum class TileStatus {
    Downloaded,
    ChildrenPending,
    Skipped,
    Absent,
};

class Downloader {
public:
    Downloader(const TileUrlBuilder &url_builder, HttpClient http, std::filesystem::path output_directory,
               std::optional<unsigned int> max_zoom_level, unsigned root_zoom_level)
        : _url_builder(url_builder),
          _output_directory(std::move(output_directory)),
          _http(std::move(http)),
          _logger(root_zoom_level),
          _max_zoom_level(max_zoom_level) {}

    // Downloads the tile `root_id` and every tile below it.
    // A tile that cannot be downloaded leaves its ancestors pending, and the tiles beside it are still downloaded.
    [[nodiscard]] alp::Result<void, DownloadError> download_recursive(const radix::tile::Id &root_id) {
        auto progress_session = this->_logger.start();
        ALP_TRY(const size_t failed_tile_count, this->download_recursive_core(root_id));
        progress_session.finish();
        if (failed_tile_count > 0) {
            return alp::make_error(IncompleteDownload{failed_tile_count});
        }
        return {};
    }

private:
    static constexpr int max_attempts = 100;

    const TileUrlBuilder &_url_builder;
    std::filesystem::path _output_directory;
    HttpClient _http;
    TileLogger _logger;
    std::optional<unsigned int> _max_zoom_level;

    // Downloads the tile `root_id` and every tile below it, and marks every tile whose subtree is complete.
    // Gives the number of tiles at or below `root_id` that could not be downloaded.
    [[nodiscard]] alp::Result<size_t, io::FileError> download_recursive_core(const radix::tile::Id &root_id) {
        auto status_result = this->download_tile(root_id);
        if (!status_result.has_value()) {
            // A local file error would fail every other tile as well, so it ends the download.
            ALP_TRY(const FetchTileError fetch_error, std::move(status_result).error().split<FetchTileError>());
            LOG_WARN_REPORT(fetch_error);
            this->_logger.missing(root_id);
            return 1;
        }
        const TileStatus status = *status_result;

        if (status == TileStatus::Skipped) {
            this->_logger.skipped(root_id);
            return 0;
        }

        if (status == TileStatus::Absent) {
            this->_logger.missing(root_id);
            return 0;
        }

        size_t failed_tile_count = 0;
        const auto children = root_id.children();
        for (size_t i = 0; i < children.size(); i++) {
            if (this->_max_zoom_level.has_value() && children[i].zoom_level > *this->_max_zoom_level) {
                this->_logger.skipped(children[i]);
                continue;
            }

            ALP_TRY(const size_t child_failed_tile_count, this->download_recursive_core(children[i]));
            failed_tile_count += child_failed_tile_count;
        }

        if (failed_tile_count == 0) {
            const std::filesystem::path path = this->tile_path(root_id);
            ALP_TRY(details::mark_tile_children_complete(path));
        }
        return failed_tile_count;
    }

    static bool is_image(const TransferInfo &info) {
        return info.content_type.starts_with("image");
    }

    std::filesystem::path tile_path(const radix::tile::Id &tile) const {
        return make_google_tile_path(_output_directory, tile, ".jpeg");
    }

    [[nodiscard]] alp::Result<TileStatus, io::FileError, FetchTileError> download_tile(const radix::tile::Id &tile) {
        const std::filesystem::path path = this->tile_path(tile);
        const std::filesystem::path pending_path = details::children_pending_tile_path(path);

        ALP_TRY_CONTEXT(const bool is_complete, io::filesystem::exists(path), "checking for a complete tile");
        if (is_complete) {
            return TileStatus::Skipped;
        }
        ALP_TRY_CONTEXT(const bool is_pending, io::filesystem::exists(pending_path), "checking for a pending tile");
        if (is_pending) {
            return TileStatus::ChildrenPending;
        }

        const std::string url = this->_url_builder.build_url(tile);
        const auto timed_out = [](const alp::Result<HttpResponse, GetError> &r) { return !r.has_value() && r.error().holds<TimedOut>(); };
        auto response_result = this->_http.get(url);
        int attempts = 1;
        while (attempts < max_attempts && timed_out(response_result)) {
            response_result = this->_http.get(url);
            attempts++;
        }

        if (!response_result.has_value()) {
            GetError error = std::move(response_result).error();
            if (error.holds_code(StatusError{404})) {
                return TileStatus::Absent;
            }
            if (attempts > 1) {
                return std::move(error).context(fmt::format("attempt {} of {}", attempts, max_attempts));
            }
            return alp::propagate(std::move(error));
        }
        const HttpResponse &response = response_result.value();
        if (!is_image(response.info)) {
            return alp::make_error(BadContentType{response.info.content_type});
        }
        ALP_TRY(details::write_file_children_pending(path, response.body));
        return TileStatus::Downloaded;
    }
};

} // namespace tile_downloader
