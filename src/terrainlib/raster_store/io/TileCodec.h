/*****************************************************************************
 * AlpineMaps.org
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

#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>

#include "io/byte.h"
#include "io/envelope.h"
#include "io/image.h"
#include "raster_store/Tile.h"
#include "raster_store/io/float_predictor.h"
#include "raster_store/pixel.h"
#include "store/Codec.h"

namespace raster_store::io {

namespace tile_codec {

    namespace detail::v1 {

        struct RasterTile {
            std::uint32_t width;
            std::uint32_t height;
            // Native bytes for AMORT, JPEG XL for AMOIT, JPEG for AMOJT, Predictor 3 bytes for AMOFT.
            ::io::byte::vector data;
            ::io::byte::vector source_attribution;
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

} // namespace tile_codec

template <typename PixelType, tile_codec::DataEncoding Encoding = tile_codec::DataEncoding::Native>
class TileCodec final : public store::Codec<Tile<PixelType>> {
    static_assert(sizeof(pixel::detail::Format<PixelType>) > 0);
    static constexpr bool is_jpeg = Encoding == tile_codec::DataEncoding::Jpeg;
    static constexpr bool is_image = is_jpeg || Encoding == tile_codec::DataEncoding::JpegXl;
    static_assert(!is_image || std::is_same_v<PixelType, glm::u8vec3>, "AMOIT and AMOJT store only RGB8 tiles");
    static constexpr bool is_float = Encoding == tile_codec::DataEncoding::FloatPredictor;
    static_assert(!is_float || (std::is_same_v<PixelType, float> && sizeof(float) == 4 && std::numeric_limits<float>::is_iec559),
        "AMOFT stores only IEEE 754 Float32 tiles");
    using Schema = std::conditional_t<is_jpeg,
        tile_codec::JpegTileSchema,
        std::conditional_t<is_image, tile_codec::ImageTileSchema, std::conditional_t<is_float, tile_codec::FloatTileSchema, tile_codec::TileSchema>>>;
    static constexpr std::string_view name = is_jpeg ? "AMOJT" : is_image ? "AMOIT" : is_float ? "AMOFT" : "AMORT";

public:
    explicit TileCodec(const glm::uvec2 dimensions = glm::uvec2(default_tile_side),
        const ::io::envelope::CompressionAlgorithm compression_algorithm = ::io::envelope::CompressionAlgorithm::ZstdDefaultCompression)
        : m_dimensions(dimensions)
        , m_compression_algorithm(compression_algorithm)
    {
    }

    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override
    {
        return { this->add_extension(node_path, is_jpeg ? ".amojt" : is_image ? ".amoit" : is_float ? ".amoft" : ".amort") };
    }

    Expected<Tile<PixelType>> read(
        const std::filesystem::path& node_path, const std::optional<store::NodeDataHashView> expected_hash = std::nullopt) const override
    {
        if (auto valid = validate_dimensions(m_dimensions); !valid) {
            return Error::propagate(std::move(valid));
        }
        auto encoded = ::io::envelope::read<Schema>(paths(node_path).front(), expected_hash);
        if (!encoded) {
            return Error::propagate(std::move(encoded), "read " + std::string(name) + " tile");
        }
        if (glm::uvec2(encoded->width, encoded->height) != m_dimensions) {
            return Error::fail(Error::Code::CorruptData, std::string(name) + " tile dimensions do not match snapshot metadata");
        }
        const std::uint64_t count = std::uint64_t { encoded->width } * encoded->height;
        if ((!is_image && (encoded->data.size() % sizeof(PixelType) != 0 || encoded->data.size() / sizeof(PixelType) != count))
            || encoded->source_attribution.size() % sizeof(std::uint16_t) != 0
            || encoded->source_attribution.size() / sizeof(std::uint16_t) != count) {
            return Error::fail(Error::Code::CorruptData, std::string(name) + " raster buffer byte counts do not match dimensions and payload type");
        }
        Tile<PixelType> tile(encoded->width);
        if constexpr (is_image) {
            auto data = ::io::image::decode_rgb8(encoded->data, m_dimensions);
            if (!data) {
                return Error::propagate(std::move(data), "decode " + std::string(name) + " tile data");
            }
            tile.data = std::move(*data);
        } else {
            if constexpr (is_float) {
                float_predictor::decode(encoded->data, m_dimensions.x);
            }
            std::memcpy(tile.data.bytes().data(), encoded->data.data(), encoded->data.size());
        }
        std::memcpy(tile.attribution.bytes().data(), encoded->source_attribution.data(), encoded->source_attribution.size());
        return tile;
    }

    Expected<store::NodeDataHash> write(const std::filesystem::path& node_path, const Tile<PixelType>& tile) const override
    {
        if (auto valid = validate_dimensions(m_dimensions); !valid) {
            return Error::propagate(std::move(valid));
        }
        if (tile.data.size() != m_dimensions || tile.attribution.size() != m_dimensions) {
            return Error::fail(Error::Code::InvalidInput, "both tile rasters must match snapshot dimensions");
        }
        const auto source_attribution = tile.attribution.bytes();
        tile_codec::detail::v1::RasterTile encoded { m_dimensions.x, m_dimensions.y, {}, { source_attribution.begin(), source_attribution.end() } };
        if constexpr (is_image) {
            auto data = ::io::image::encode(tile.data, is_jpeg ? ::io::image::Format::Jpeg : ::io::image::Format::JpegXl);
            if (!data) {
                return Error::propagate(std::move(data), "encode " + std::string(name) + " tile data");
            }
            encoded.data = std::move(*data);
        } else {
            const auto data = tile.data.bytes();
            encoded.data.assign(data.begin(), data.end());
            if constexpr (is_float) {
                float_predictor::encode(encoded.data, m_dimensions.x);
            }
        }
        return ::io::envelope::write<Schema>(encoded, paths(node_path).front(), ::io::byte::WriteMode::CreateNew, true, m_compression_algorithm);
    }

    bool supports_hashing() const override { return true; }

private:
    glm::uvec2 m_dimensions;
    ::io::envelope::CompressionAlgorithm m_compression_algorithm;
};

namespace tile_codec {

    template <typename PixelType>
    Expected<std::unique_ptr<store::Codec<Tile<PixelType>>>> from_name(const std::string_view name,
        const glm::uvec2 dimensions,
        const ::io::envelope::CompressionAlgorithm compression_algorithm = ::io::envelope::CompressionAlgorithm::ZstdDefaultCompression)
    {
        if (name == "amort") {
            return std::make_unique<TileCodec<PixelType>>(dimensions, compression_algorithm);
        }
        if (name == "amoit" || name == "amojt") {
            if constexpr (std::is_same_v<PixelType, glm::u8vec3>) {
                if (name == "amojt") {
                    return std::make_unique<TileCodec<PixelType, DataEncoding::Jpeg>>(dimensions, compression_algorithm);
                }
                return std::make_unique<TileCodec<PixelType, DataEncoding::JpegXl>>(dimensions, compression_algorithm);
            } else {
                return Error::fail(
                    Error::Code::Unsupported, "the " + std::string(name) + " raster codec requires " + pixel::identifier<glm::u8vec3>() + " pixels");
            }
        }
        if (name == "amoft") {
            if constexpr (std::is_same_v<PixelType, float>) {
                return std::make_unique<TileCodec<PixelType, DataEncoding::FloatPredictor>>(dimensions, compression_algorithm);
            } else {
                return Error::fail(Error::Code::Unsupported, "the amoft raster codec requires " + pixel::identifier<float>() + " pixels");
            }
        }
        return Error::fail(Error::Code::Unsupported, "unsupported raster codec selector: " + std::string(name));
    }

} // namespace tile_codec

} // namespace raster_store::io
