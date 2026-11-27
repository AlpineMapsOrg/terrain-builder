/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2022 Adam Celarek-Litofcenko
 * Copyright (C) 2022 Martin Braunsperger
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

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>

#include <fmt/format.h>
#include <gdal.h>
#include <glm/gtc/type_precision.hpp>
#include <radix/raster.h>
#include <radix/tile.h>

#include "common_error_codes.h"
#include "containers/UniqueResource.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "gis/Dataset.h"
#include "gis/srs/Transform.h"

namespace gis {

enum class Projection { WebMercator, Geographic };

// A band number the dataset does not have.
struct MissingBand {
    // The one-based band number.
    uint32_t band;
    // How many bands the dataset has.
    uint32_t band_count;

    bool operator==(const MissingBand &) const = default;
};

inline std::string format_as(const MissingBand &error) {
    return fmt::format("the dataset has no band {}, it has {}", error.band, error.band_count);
}

// A band whose data type the reader cannot read.
struct UnsupportedBandType {
    // The one-based band number.
    uint32_t band;
    // The data type of the band.
    GDALDataType type;
    // The data type the reader requires.
    GDALDataType required;

    bool operator==(const UnsupportedBandType &) const = default;
};

inline std::string format_as(const UnsupportedBandType &error) {
    return fmt::format("band {} has type {}, but the reader requires {}", error.band, GDALGetDataTypeName(error.type), GDALGetDataTypeName(error.required));
}

// GDAL cannot reproject the dataset into the target projection.
struct UnsupportedReprojection : alp::MessageCode<"the dataset cannot be reprojected into the target projection"> {};

// A band whose mask the reader cannot read.
struct UnsupportedMaskType {
    // The one-based band number.
    uint32_t band;

    bool operator==(const UnsupportedMaskType &) const = default;
};

inline std::string format_as(const UnsupportedMaskType &error) {
    return fmt::format("the mask of band {} is not of type Byte", error.band);
}

// A GDAL driver that is not registered.
struct MissingDriver {
    // The short name of the driver.
    std::string name;

    bool operator==(const MissingDriver &) const = default;
};

inline std::string format_as(const MissingDriver &error) {
    return fmt::format("GDAL has no {} driver registered", error.name);
}

// GDAL failed to read a window of the dataset.
struct WindowReadError : alp::MessageCode<"the window cannot be read from the dataset"> {};

// The bounds do not overlap the dataset's coverage.
struct OutsideCoverage : alp::MessageCode<"the bounds lie outside the dataset's coverage"> {};

// Reads selected dataset bands into a target projection, using Lanczos at base resolution and GDAL's approximate transformer.
// Validity comes from the effective GDAL masks of the selected bands; RGB bands share one combined mask.
// Sources must be Float32 for float and Byte for RGB; there is no type conversion.
// A reader and its dataset belong to one thread at a time.
template <typename Pixel>
class DatasetReader {
    static_assert(std::is_same_v<Pixel, float> || std::is_same_v<Pixel, glm::u8vec3>, "DatasetReader supports float and glm::u8vec3 pixels");

public:
    using MakeError = alp::Error<MissingBand, UnsupportedBandType, Dataset::ReadGeoreferencingKindError, MissingSrs, Dataset::ReadGeoTransformError, UnsupportedReprojection, OutOfMemory>;
    using ReadError = alp::Error<UnsupportedMaskType, MissingDriver, WindowReadError, OutOfMemory>;
    using PixelSizeError = alp::Error<Dataset::ComputeCoverageError, OutsideCoverage, MissingSrs, srs::TransformError, Dataset::ReadGeoTransformError>;

    static constexpr uint32_t channel_count = std::is_same_v<Pixel, float> ? 1 : 3;

    // Data and validity have the same size.
    // Validity is zero for invalid and nonzero for valid pixels; it is not image opacity.
    // Invalid pixels hold the default pixel, which valid pixels may equal as well.
    struct Samples {
        radix::Raster<Pixel> data;
        radix::Raster<uint8_t> valid;
    };

    // Creates a reader of `bands` of `dataset` in `projection`.
    // Bands are one-based GDAL band numbers, in RGB order for colour; the default pixel of a float reader must be finite.
    // Fails if a band is missing or has another type, if the dataset has no or several kinds of georeferencing or no spatial reference system, or if GDAL cannot reproject it.
    [[nodiscard]]
    static alp::Result<DatasetReader, MakeError> make(std::shared_ptr<Dataset> dataset, Projection projection, std::array<uint32_t, channel_count> bands, Pixel default_pixel);

    // Reads the window `bounds`, in the reader's projection, at `size` pixels.
    // Bounds must be finite and nonempty; windows without valid source coverage yield default pixels and zero validity.
    // Fails if a band's mask is not of type Byte or GDAL fails to read the window.
    [[nodiscard]]
    alp::Result<Samples, ReadError> read(const radix::tile::SrsBounds &bounds, glm::uvec2 size);

    // Computes the smallest source pixel size within `bounds`, per axis, in units of the projection.
    // Approximate: sampled on a grid within the dataset coverage.
    // Fails if the bounds lie outside the coverage or a sample cannot be mapped into the dataset's system.
    [[nodiscard]]
    alp::Result<glm::dvec2, PixelSizeError> min_pixel_size(const radix::tile::SrsBounds &bounds) const;

    // The projection this reader reads into.
    [[nodiscard]]
    Projection projection() const {
        return this->m_projection;
    }

private:
    // Opaque handle of GDAL's image-to-image transformer from the output pixels to the dataset pixels.
    struct Transformer;
    struct TransformerDeleter {
        void operator()(Transformer *transformer) const noexcept;
    };

    DatasetReader(std::shared_ptr<Dataset> dataset, Projection projection, std::array<uint32_t, channel_count> bands, Pixel default_pixel, UniquePointer<Transformer, TransformerDeleter> transformer);

    std::shared_ptr<Dataset> m_dataset;
    Projection m_projection;
    std::array<uint32_t, channel_count> m_bands;
    Pixel m_default_pixel;
    UniquePointer<Transformer, TransformerDeleter> m_transformer;
};

extern template class DatasetReader<float>;
extern template class DatasetReader<glm::u8vec3>;

} // namespace gis
