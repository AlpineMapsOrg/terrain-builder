/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2024 Martin Braunsperger
 * Copyright (C) 2024 Adam Celarek-Litofcenko
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

#include <filesystem>
#include <optional>

#include <fmt/format.h>
#include <opencv2/opencv.hpp>
#include <radix/tile.h>

#include "error/Error.h"
#include "error/Result.h"
#include "error/log.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/FileError.h"
#include "io/filesystem.h"
#include "io/mat.h"
#include "tile_path.h"

// A tile that a provider does not have.
struct MissingTile {
    radix::tile::Id tile;

    bool operator==(const MissingTile &) const = default;
};

inline std::string format_as(const MissingTile &error) {
    return fmt::format("the provider has no tile {}", radix::tile::to_string(error.tile));
}

// A tile that a provider has, but whose image it cannot read.
struct UnreadableTile {
    radix::tile::Id tile;

    bool operator==(const UnreadableTile&) const = default;
};

inline std::string format_as(const UnreadableTile& error) {
    return fmt::format("the provider cannot read the image of tile {}", radix::tile::to_string(error.tile));
}

// An error from getting a tile from a provider.
using TileError = alp::Error<MissingTile, UnreadableTile>;

class TileProvider {
public:
    virtual ~TileProvider() = default;
    virtual bool has_tile(const radix::tile::Id tile) const = 0;
    virtual alp::Result<cv::Mat, TileError> get_tile(const radix::tile::Id tile) const = 0;
};

class TilePathProvider : public TileProvider {
public:
    virtual std::optional<std::filesystem::path> get_tile_path(const radix::tile::Id tile) const = 0;

    alp::Result<cv::Mat, TileError> get_tile(const radix::tile::Id tile) const override {
        const std::optional<std::filesystem::path> tile_path = this->get_tile_path(tile);
        if (!tile_path.has_value()) {
            return alp::make_error(MissingTile{tile});
        }

        auto usable_result = TilePathProvider::is_usable_tile_path(tile_path.value());
        ALP_TRY(const bool usable, alp::map_error<io::FileError>(std::move(usable_result), UnreadableTile{tile}));
        if (!usable) {
            return alp::make_error(MissingTile{tile});
        }

        auto image_result = io::mat::read(tile_path.value());
        ALP_TRY(cv::Mat image, alp::map_error<io::mat::ReadError>(std::move(image_result), UnreadableTile{tile}));
        return image;
    }
    virtual bool has_tile(const radix::tile::Id tile) const override {
        const std::optional<std::filesystem::path> tile_path = this->get_tile_path(tile);
        if (!tile_path.has_value()) {
            return false;
        }
        const alp::Result<bool, io::FileError> usable_result = TilePathProvider::is_usable_tile_path(tile_path.value());
        LOG_WARN_IF_ERROR(usable_result, "checking for a tile file, treating it as missing");
        return usable_result.value_or(false);
    }

private:
    static alp::Result<bool, io::FileError> is_usable_tile_path(const std::filesystem::path &tile_path) {
        if (tile_path.empty()) {
            return false;
        }
        return io::filesystem::is_regular_file(tile_path);
    }
};

class StaticTileProvider : public TileProvider {
public:
    std::unordered_map<radix::tile::Id, cv::Mat, radix::tile::Id::Hasher> tiles;

    StaticTileProvider(const std::unordered_map<radix::tile::Id, cv::Mat, radix::tile::Id::Hasher>& tiles)
        : tiles(tiles) {
    }

    virtual alp::Result<cv::Mat, TileError> get_tile(const radix::tile::Id tile_id) const override {
        const auto tile = this->tiles.find(tile_id);
        if (tile != this->tiles.end()) {
            return tile->second;
        } else {
            return alp::make_error(MissingTile{tile_id});
        }
    }

    virtual bool has_tile(const radix::tile::Id tile_id) const override {
        return this->tiles.find(tile_id) != this->tiles.cend();
    }
};

class EmptyTileProvider final : public TileProvider {
public:
    virtual alp::Result<cv::Mat, TileError> get_tile(const radix::tile::Id tile_id) const override {
        return alp::make_error(MissingTile{tile_id});
    }

    virtual bool has_tile(const radix::tile::Id) const override {
        return false;
    }
};

template <typename Inner>
class ZoomRangeTileProvider final : public TileProvider {
public:
    template <typename P>
    ZoomRangeTileProvider(
        P &&inner,
        std::optional<uint32_t> min_zoom,
        std::optional<uint32_t> max_zoom)
        : _inner(std::move(inner)),
          _min_zoom(min_zoom.value_or(std::numeric_limits<uint32_t>::min())),
          _max_zoom(max_zoom.value_or(std::numeric_limits<uint32_t>::max())) {}

    alp::Result<cv::Mat, TileError> get_tile(const radix::tile::Id tile_id) const override {
        if (!this->is_tile_in_range(tile_id)) {
            return alp::make_error(MissingTile{tile_id});
        }

        return this->inner().get_tile(tile_id);
    }

    bool has_tile(const radix::tile::Id tile_id) const override {
        if (!this->is_tile_in_range(tile_id)) {
            return false;
        }

        return this->inner().has_tile(tile_id);
    }

private:
    bool is_tile_in_range(const radix::tile::Id tile_id) const {
        if (tile_id.zoom_level < this->_min_zoom) {
            return false;
        }

        if (tile_id.zoom_level > this->_max_zoom) {
            return false;
        }

        return true;
    }

    const TileProvider &inner() const {
        if constexpr (requires { *this->_inner; }) { // pointers
            return *this->_inner;
        } else { // value
            return this->_inner;
        }
    }

    Inner _inner;
    uint32_t _min_zoom;
    uint32_t _max_zoom;
};

class GoogleMapboxTilePathProvider : public TilePathProvider {
public:
    GoogleMapboxTilePathProvider(std::filesystem::path base_path)
        : base_path(base_path) {}

    std::optional<std::filesystem::path> get_tile_path(const radix::tile::Id tile_id) const override {
        return make_google_tile_path(base_path, tile_id, ".jpeg");
    }

private:
    std::filesystem::path base_path;
};
