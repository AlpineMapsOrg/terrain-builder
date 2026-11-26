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

#pragma once

#include <bit>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "common_error_codes.h"
#include "error/Result.h"
#include "error/try.h"
#include "io/WriteMode.h"
#include "io/bytes.h"
#include "io/compression.h"
#include "io/envelope.h"
#include "io/image.h"
#include "numeric/checked.h"
#include "path_utils.h"
#include "raster_store/Tile.h"
#include "raster_store/io/float_predictor.h"
#include "raster_store/pixel.h"
#include "store/Codec.h"
#include "store/envelope_utils.h"

namespace raster_store::io {

namespace tile_codec {

namespace detail::v1 {

struct RasterTile {
    uint32_t width;
    uint32_t height;
    // Native element bytes for AMORT, a JPEG XL codestream for AMOIT, a JPEG image for AMOJT, TIFF Predictor 3 bytes for AMOFT.
    ::io::bytes::vector data;
    ::io::bytes::vector source_attribution;
};

} // namespace detail::v1

using TileSchema = ::io::envelope::PayloadSchema<"raster_store.Tile", ::io::envelope::Version<1, detail::v1::RasterTile>>;
using ImageTileSchema = ::io::envelope::PayloadSchema<"raster_store.ImageTile", ::io::envelope::Version<1, detail::v1::RasterTile>>;
using JpegTileSchema = ::io::envelope::PayloadSchema<"raster_store.JpegTile", ::io::envelope::Version<1, detail::v1::RasterTile>>;
using FloatTileSchema = ::io::envelope::PayloadSchema<"raster_store.FloatTile", ::io::envelope::Version<1, detail::v1::RasterTile>>;

// Storage of the data raster; attribution is always stored losslessly.
enum class DataEncoding {
    Native, // AMORT: lossless native element bytes of any pixel type.
    JpegXl, // AMOIT: lossy JPEG XL image of RGB8 pixels.
    FloatPredictor, // AMOFT: lossless TIFF Predictor 3 bytes of Float32 pixels.
    Jpeg, // AMOJT: lossy JPEG image of RGB8 pixels.
};

inline constexpr std::string_view amort_id = "amort";
inline constexpr std::string_view amoit_id = "amoit";
inline constexpr std::string_view amoft_id = "amoft";
inline constexpr std::string_view amojt_id = "amojt";

namespace detail {

// The failure reading a tile for a failure decoding its image data, which it keeps as its source.
[[nodiscard]]
inline store::CodecReadError to_codec_read_error(::io::image::DecodeError error) {
    return std::move(error).map<::io::image::CorruptData>(store::CorruptData{}).map<::io::image::Unknown>(store::Unknown{});
}

// The failure writing a tile for a failure encoding its data as an image, which it keeps as its source.
[[nodiscard]]
inline store::CodecWriteError to_codec_write_error(::io::image::EncodeError error) {
    return std::move(error).map<::io::image::Unknown>(store::Unknown{});
}

} // namespace detail

} // namespace tile_codec

template <typename PixelType, tile_codec::DataEncoding Encoding = tile_codec::DataEncoding::Native>
class TileCodec final : public store::Codec<Tile<PixelType>> {
    static_assert(sizeof(pixel::detail::Format<PixelType>) > 0);
    static constexpr bool is_jpeg = Encoding == tile_codec::DataEncoding::Jpeg;
    static constexpr bool is_image = is_jpeg || Encoding == tile_codec::DataEncoding::JpegXl;
    static_assert(!is_image || std::is_same_v<PixelType, glm::u8vec3>, "AMOIT and AMOJT store only RGB8 tiles");
    static constexpr bool is_float = Encoding == tile_codec::DataEncoding::FloatPredictor;
    static_assert(!is_float || (std::is_same_v<PixelType, float> && sizeof(float) == 4 && std::numeric_limits<float>::is_iec559), "AMOFT stores only IEEE 754 Float32 tiles");
    static_assert(std::endian::native == std::endian::little, "AMORT stores raster elements in little-endian order");
    using Schema = std::conditional_t<is_jpeg, tile_codec::JpegTileSchema, std::conditional_t<is_image, tile_codec::ImageTileSchema, std::conditional_t<is_float, tile_codec::FloatTileSchema, tile_codec::TileSchema>>>;
    static constexpr std::string_view codec_id = is_jpeg ? tile_codec::amojt_id : is_image ? tile_codec::amoit_id : is_float ? tile_codec::amoft_id : tile_codec::amort_id;
    static constexpr std::string_view extension = is_jpeg ? ".amojt" : is_image ? ".amoit" : is_float ? ".amoft" : ".amort";

public:
    // `side` must be positive.
    explicit TileCodec(const uint32_t side = default_tile_side, const ::io::compression::Algorithm compression_algorithm = ::io::compression::Algorithm::ZstdDefault)
        : _side(side),
          _compression_algorithm(compression_algorithm) {
        ASSERT(side > 0u);
    }

    std::string_view id() const override { return codec_id; }

    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override { return { this->file_path(node_path) }; }

    [[nodiscard]]
    alp::Result<Tile<PixelType>, store::CodecReadError> read(const std::filesystem::path& node_path, const std::optional<store::PayloadHashView> expected_hash = std::nullopt) const override {
        auto read_result = ::io::envelope::read<Schema>(this->file_path(node_path), {.expected_hash = expected_hash}).transform_error(store::to_codec_read_error);
        ALP_TRY(const tile_codec::detail::v1::RasterTile encoded, std::move(read_result));

        if (encoded.width != this->_side || encoded.height != this->_side) {
            return alp::make_error(store::CorruptData{}, fmt::format("the tile is {}x{} pixels, not {}x{}", encoded.width, encoded.height, this->_side, this->_side));
        }
        const uint64_t pixel_count = uint64_t{encoded.width} * encoded.height;
        if (!is_image && checked_mul<uint64_t>(pixel_count, sizeof(PixelType)) != encoded.data.size()) {
            return alp::make_error(store::CorruptData{}, fmt::format("the tile's data has {} bytes, not {} pixels of {} bytes", encoded.data.size(), pixel_count, sizeof(PixelType)));
        }
        if (checked_mul<uint64_t>(pixel_count, sizeof(uint16_t)) != encoded.source_attribution.size()) {
            return alp::make_error(store::CorruptData{}, fmt::format("the tile's attribution has {} bytes, not {} pixels of {} bytes", encoded.source_attribution.size(), pixel_count, sizeof(uint16_t)));
        }

        Tile<PixelType> tile(this->_side);
        if constexpr (is_image) {
            const ::io::image::DecodeOptions options{
                .expected_size = glm::uvec2(this->_side),
            };
            auto decode_result = ::io::image::decode<::io::image::RGB8>(encoded.data, options).transform_error(tile_codec::detail::to_codec_read_error);
            ALP_TRY(::io::image::RGB8 data, std::move(decode_result));
            tile.data = std::move(data);
        } else {
            std::memcpy(tile.data.bytes().data(), encoded.data.data(), encoded.data.size());
            if constexpr (is_float) {
                float_predictor::decode(std::as_writable_bytes(tile.data.bytes()), this->_side);
            }
        }
        std::memcpy(tile.attribution.bytes().data(), encoded.source_attribution.data(), encoded.source_attribution.size());

        return tile;
    }

    [[nodiscard]]
    alp::Result<store::PayloadHash, store::CodecWriteError> write(const std::filesystem::path& node_path, const Tile<PixelType>& tile) const override {
        ASSERT(tile.data.size() == glm::uvec2(this->_side) && tile.attribution.size() == glm::uvec2(this->_side), "both rasters of the tile have the codec's side");

        tile_codec::detail::v1::RasterTile encoded{
            .width = this->_side,
            .height = this->_side,
            .data = {},
            .source_attribution = ::io::bytes::vector(tile.attribution.size_in_bytes()),
        };
        std::memcpy(encoded.source_attribution.data(), tile.attribution.bytes().data(), encoded.source_attribution.size());
        if constexpr (is_image) {
            const ::io::image::Format format = is_jpeg ? ::io::image::Format::Jpeg : ::io::image::Format::JpegXl;
            auto encode_result = ::io::image::encode(tile.data, format).transform_error(tile_codec::detail::to_codec_write_error);
            ALP_TRY(::io::bytes::vector data, std::move(encode_result));
            encoded.data = std::move(data);
        } else {
            encoded.data.resize(tile.data.size_in_bytes());
            std::memcpy(encoded.data.data(), tile.data.bytes().data(), encoded.data.size());
            if constexpr (is_float) {
                float_predictor::encode(std::as_writable_bytes(std::span(encoded.data)), this->_side);
            }
        }

        const ::io::envelope::WriteOptions options{
            .mode = ::io::WriteMode::CreateNew,
            .compression_algorithm = this->_compression_algorithm,
        };
        auto write_result = ::io::envelope::write<Schema>(encoded, this->file_path(node_path), options).transform_error(store::to_codec_write_error);
        ALP_TRY(store::PayloadHash hash, std::move(write_result));

        return hash;
    }

    bool supports_hashing() const override { return true; }

private:
    // The file that holds the tile at `node_path`.
    std::filesystem::path file_path(const std::filesystem::path& node_path) const { return paths::with_added_extension(node_path, extension); }

    uint32_t _side;
    ::io::compression::Algorithm _compression_algorithm;
};

namespace tile_codec {

// The codec that `id` names, for tiles of `side` pixels, compressing with `compression_algorithm`.
template <typename PixelType>
[[nodiscard]]
alp::Result<std::unique_ptr<store::Codec<Tile<PixelType>>>, Unsupported> resolve(
    const std::string_view id,
    const uint32_t side,
    const ::io::compression::Algorithm compression_algorithm = ::io::compression::Algorithm::ZstdDefault
) {
    if (id == amort_id) {
        return std::make_unique<TileCodec<PixelType>>(side, compression_algorithm);
    }
    if (id == amoit_id) {
        if constexpr (std::is_same_v<PixelType, glm::u8vec3>) {
            return std::make_unique<TileCodec<PixelType, DataEncoding::JpegXl>>(side, compression_algorithm);
        } else {
            return alp::make_error(Unsupported{}, fmt::format("the codec {} stores only {} pixels", id, pixel::identifier<glm::u8vec3>()));
        }
    }
    if (id == amojt_id) {
        if constexpr (std::is_same_v<PixelType, glm::u8vec3>) {
            return std::make_unique<TileCodec<PixelType, DataEncoding::Jpeg>>(side, compression_algorithm);
        } else {
            return alp::make_error(Unsupported{}, fmt::format("the codec {} stores only {} pixels", id, pixel::identifier<glm::u8vec3>()));
        }
    }
    if (id == amoft_id) {
        if constexpr (std::is_same_v<PixelType, float>) {
            return std::make_unique<TileCodec<PixelType, DataEncoding::FloatPredictor>>(side, compression_algorithm);
        } else {
            return alp::make_error(Unsupported{}, fmt::format("the codec {} stores only {} pixels", id, pixel::identifier<float>()));
        }
    }
    return alp::make_error(Unsupported{}, fmt::format("the codec {} is unknown", id));
}

} // namespace tile_codec

} // namespace raster_store::io
