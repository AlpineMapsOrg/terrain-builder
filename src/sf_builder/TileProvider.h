#pragma once

#include <filesystem>
#include <optional>

#include <fmt/format.h>
#include <opencv2/opencv.hpp>
#include <radix/tile.h>

#include "error/Error.h"
#include "error/Result.h"
#include "error/utils.h"
#include "texture/io.h"
#include "tile_path.h"

// A tile that a provider does not have.
struct MissingTile {
    radix::tile::Id tile;

    bool operator==(const MissingTile &) const = default;
};

inline std::string format_as(const MissingTile &error) {
    return fmt::format("the provider has no tile {}", radix::tile::to_string(error.tile));
}

// An error from getting a tile from a provider.
using TileError = alp::Error<MissingTile, texture::io::ReadError>;

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
        if (!tile_path.has_value() || !TilePathProvider::is_usable_tile_path(tile_path.value())) {
            return alp::make_unexpected(MissingTile{tile});
        }
        return texture::io::read_from_path(tile_path.value());
    }
    virtual bool has_tile(const radix::tile::Id tile) const override {
        const std::optional<std::filesystem::path> tile_path = this->get_tile_path(tile);
        return tile_path.has_value() && TilePathProvider::is_usable_tile_path(tile_path.value());
    }

private:
    static bool is_usable_tile_path(const std::filesystem::path &tile_path) {
        return !tile_path.empty() && std::filesystem::is_regular_file(tile_path);
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
            return alp::make_unexpected(MissingTile{tile_id});
        }
    }

    virtual bool has_tile(const radix::tile::Id tile_id) const override {
        return this->tiles.find(tile_id) != this->tiles.cend();
    }
};

class EmptyTileProvider final : public TileProvider {
public:
    virtual alp::Result<cv::Mat, TileError> get_tile(const radix::tile::Id tile_id) const override {
        return alp::make_unexpected(MissingTile{tile_id});
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
            return alp::make_unexpected(MissingTile{tile_id});
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
