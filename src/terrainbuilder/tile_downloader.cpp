#include <charconv>
#include <filesystem>
#include <fstream>
#include <map>
#include <span>
#include <string_view>
#include <iostream>
#include <string_view>
#include <typeinfo>

#include <fmt/core.h>
#include <curl/curl.h>

#include "ParallelTiler.h"
#include "TileHeightsGenerator.h"
#include "alpine_raster.h"
#include "cesium_tin_terra.h"
#include "ctb/Grid.hpp"
#include "Dataset.h"
#include "DatasetReader.h"
#include "TopDownTiler.h"
#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"
#include "srs.h"

using namespace std::literals;

bool char_equals_ignore_case(const char a, const char b) {
    return std::tolower(std::char_traits<char>::to_int_type(a)) ==
           std::tolower(std::char_traits<char>::to_int_type(b));
}
bool string_equals_ignore_case(const std::string_view &a, const std::string_view &b) {
    return a.size() == b.size() &&
           std::equal(a.begin(), a.end(), b.begin(), char_equals_ignore_case);
}

unsigned int svtoui(const std::string_view s) {
    unsigned int n;
    std::from_chars_result res = std::from_chars(s.begin(), s.end(), n);

    // These two exceptions reflect the behavior of std::stoi.
    if (res.ec == std::errc::invalid_argument) {
        throw std::invalid_argument{"invalid_argument"};
    } else if (res.ec == std::errc::result_out_of_range) {
        throw std::out_of_range{"out_of_range"};
    }

    return n;
}

template <class K, class V>
constexpr const V& map_get_or_default(const std::map<K, V>& map, const K& key, const V& default_value) {
    const auto iter = map.find(key);
    if (iter == map.end()) {
        return default_value;
    }
    return iter->second;
}

constexpr auto &map_get_required(const auto &map, const auto &key) {
    const auto itr = map.find(key);
    return itr != map.cend() ? itr->second : throw std::runtime_error(fmt::format("missing argument \"{}\"", key));;
}

class TileUrlBuilder {
public:
    virtual std::string build_url(const tile::Id &tile_id) const = 0;
};

// https://mapsneu.wien.gv.at/basemapneu/1.0.0/WMTSCapabilities.xml
class BasemapTileUrlBuilder : public TileUrlBuilder {
public:
    BasemapTileUrlBuilder(const std::map<std::string_view, std::string_view> &args) {
        this->layer = map_get_or_default(args, "layer"sv, "bmaporthofoto30cm"sv);
        this->style = map_get_or_default(args, "style"sv, "normal"sv);
    }
    
    std::string build_url(const tile::Id &tile_id) const {
        return fmt::format("https://mapsneu.wien.gv.at/basemap/{}/{}/google3857/{}/{}/{}.jpeg", layer, style, tile_id.zoom_level, tile_id.coords.x, tile_id.coords.y);
    }

private:
    std::string_view layer;
    std::string_view style;
};

struct WriteCallbackData {
    std::ofstream file;
    std::string path;
};

// Write callback function to write downloaded data to the file
size_t write_callback(void *ptr, size_t size, size_t nmemb, void *userdata) {
    WriteCallbackData &data = *static_cast<WriteCallbackData *>(userdata);

    // Check if the file is opened
    if (!data.file.is_open()) {
        // If the file is not opened yet, try to open it
        data.file.open(data.path, std::ios::binary);
        
        if (!data.file.is_open()) {
            // Handle file opening failure
            return 0; // Returning 0 will abort the download
        }
    }

    // Write the data to the file and return the number of bytes written
    size_t total_size = size * nmemb;
    data.file.write(static_cast<char *>(ptr), total_size);

    return total_size;
}

bool download_tile_by_url(const std::string& url, const std::string& path) {
    // Skip tile if it already exists.
    if (std::filesystem::exists(path)) {
        return true;
    }

    // TODO: reuse instance
    CURL* curl = curl_easy_init();
    if (!curl) {
        return false;
    }

    // Create a custom data structure to pass file-related information to the write callback
    WriteCallbackData callback_data;

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_FAILONERROR, TRUE);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &callback_data);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1000);
    CURLcode res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    if (callback_data.file.is_open()) {
        callback_data.file.close();
    }
    if (res != CURLE_OK) {
        std::string_view message = curl_easy_strerror(res);
        std::string output = fmt::format("CURL ERROR: [{}] {}", static_cast<unsigned int>(res), message);
        std::cout << output << std::endl;
        remove(path.c_str());
        return false;
    }

    return true;
}

bool download_tile_by_id(const tile::Id root_id, const TileUrlBuilder &url_builder) {
    const std::string url = url_builder.build_url(root_id);
    return download_tile_by_url(url, fmt::format("tile-{}-{}-{}.jpeg", root_id.zoom_level, root_id.coords.x, root_id.coords.y));
}

bool download_tile_by_id_recursive(const tile::Id root_id, const TopDownTiler& tiler, const TileUrlBuilder &url_builder) {
    std::cout << root_id << std::endl;

    if (!download_tile_by_id(root_id, url_builder)) {
        return false;
    }

    const auto subtiles = tiler.generateTiles(root_id);
    for (auto &&tile : subtiles) {
        download_tile_by_id_recursive(tile.id, tiler, url_builder);
    }

    return true;
}

int main(int argc, char *argv[]) {
    // Copy args into vector for easier access.
    const std::vector<std::string_view> args(argv + 1, argv + argc);

    // Parse argument list as key-value pairs and place into map.
    std::map<std::string_view, std::string_view> arg_map;
    for (auto arg = args.begin(); arg != args.end(); arg++) {
        std::string_view key = *arg;
        if (key.starts_with("--")) {
            key = key.substr(2);
        } else if (key.starts_with("-") || key.starts_with("/")) {
            key = key.substr(1);
        } else {
            throw std::runtime_error(fmt::format("unexpected value \"{}\"", key));
        }

        if (arg + 1 == args.end()) {
            throw std::runtime_error(fmt::format("no value for key \"{}\"", key));
        }
        arg += 1;
        const std::string_view value = *arg;

        arg_map.emplace(key, value);
    }

    // Check provider and create corresponding url builder
    if (!arg_map.contains("provider")) {
        throw std::runtime_error("no tile provider given");
    }
    const std::string_view provider = arg_map.at("provider");

    std::unique_ptr<TileUrlBuilder> url_builder;
    if (string_equals_ignore_case(provider, "basemap")) {
        url_builder = std::make_unique<BasemapTileUrlBuilder>(arg_map);
    } else {
        throw std::runtime_error(fmt::format("unsupported tile provider \"{}\"", provider));
    }

    // Parse spatial reference system and scheme
    const unsigned int srs = svtoui(map_get_or_default(arg_map, "srs"sv, "3857"sv));
    const std::string_view scheme_str = map_get_or_default(arg_map, "scheme"sv, "SlippyMap"sv);
    tile::Scheme scheme;
    if (string_equals_ignore_case(scheme_str, "SlippyMap")
        || string_equals_ignore_case(scheme_str, "Google")
        || string_equals_ignore_case(scheme_str, "XYZ")) {
        scheme = tile::Scheme::SlippyMap;
    } else if (string_equals_ignore_case(scheme_str, "TMS")) {
        scheme = tile::Scheme::Tms;
    } else {
        throw std::runtime_error(fmt::format("unsupported srs scheme \"{}\"", scheme_str));
    }

    if (srs != static_cast<int>(ctb::Grid::Srs::SphericalMercator)) {
        throw std::runtime_error(fmt::format("unsupported srs EPSG \"{}\"", srs));
    }
    
    // Construct tiler and root tile
    const auto grid = ctb::GlobalMercator();
    const TopDownTiler tiler(grid, grid.getExtent(), tile::Border::No, scheme);

    const unsigned int zoom = svtoui(map_get_required(arg_map, "zoom"));
    const unsigned int row = svtoui(map_get_required(arg_map, "row"));
    const unsigned int col = svtoui(map_get_required(arg_map, "col"));
    const tile::Id root_id = {zoom, {row, col}, tiler.scheme()};

    // Download tile and subtiles recursively.
    download_tile_by_id_recursive(root_id, tiler, *url_builder);
}
