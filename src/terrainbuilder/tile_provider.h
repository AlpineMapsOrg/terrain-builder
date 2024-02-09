#ifndef TILEPROVIDER_H
#define TILEPROVIDER_H

#include <filesystem>
#include <optional>

#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>
#include <radix/tile.h>

class TileProvider {
public:
    virtual std::optional<cv::Mat> get_tile(const tile::Id tile) const = 0;
    virtual bool has_tile(const tile::Id tile) const {
        return this->get_tile(tile).has_value();
    }
};

class TilePathProvider : public TileProvider {
public:
    virtual std::optional<std::filesystem::path> get_tile_path(const tile::Id tile) const = 0;

    std::optional<cv::Mat> get_tile(const tile::Id tile) const override {
        const std::optional<std::filesystem::path> tile_path = this->get_tile_path(tile);
        if (tile_path.has_value() && TilePathProvider::is_usable_tile_path(tile_path.value())) {
            return cv::imread(tile_path.value());
        }
        return std::nullopt;
    }
    virtual bool has_tile(const tile::Id tile) const override {
        const std::optional<std::filesystem::path> tile_path = this->get_tile_path(tile);
        return tile_path.has_value() && TilePathProvider::is_usable_tile_path(tile_path.value());
    }

    virtual std::optional<cv::Mat> load_tile_from_path(const std::filesystem::path& tile_path) const {
        if (TilePathProvider::is_usable_tile_path(tile_path)) {
            return cv::imread(tile_path);
        }
        return std::nullopt;
    }

private:
    static bool is_usable_tile_path(const std::filesystem::path &tile_path) {
        return !tile_path.empty() && std::filesystem::exists(tile_path);
    }
};

// TODO: Move this to radix
template<> struct std::hash<tile::Id> {
    std::size_t operator()(const tile::Id& tile) const noexcept {
        // TODO: fix
        return tile.coords.x;
    }
};

class MapBasedTileProvider : public TileProvider {
public:
    const std::unordered_map<tile::Id, cv::Mat> tiles;

    MapBasedTileProvider(std::unordered_map<tile::Id, cv::Mat> tiles) : tiles(tiles) {}

    virtual std::optional<cv::Mat> get_tile(const tile::Id tile_id) const override {
        const auto tile = this->tiles.find(tile_id);
        if (tile != this->tiles.end()) {
            return tile->second;
        } else {
            return std::nullopt;
        }
    }
    
    virtual bool has_tile(const tile::Id tile) const override {
        return this->tiles.find(tile) != this->tiles.cend();
    }
};

#endif
