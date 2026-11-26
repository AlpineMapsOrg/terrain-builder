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

#include <concepts>
#include <filesystem>
#include <optional>

#include <glm/gtc/type_precision.hpp>
#include <radix/raster.h>

#include "common_error_codes.h"
#include "error/Error.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "io/WriteMode.h"
#include "io/alloc.h"
#include "io/bytes.h"
#include "io/image/Format.h"
#include "io/limits.h"

namespace io::image {

// How `encode` compresses each format.
struct EncodeOptions {
    // JPEG quality, from 0 to 100.
    int jpeg_quality = 95;
    // PNG compression level, from 0 to 9.
    int png_compression = 1;
    // Butteraugli distance, from 0 to 25, where 0 selects lossless coding.
    float jpegxl_distance = 0.6f;
    // JPEG XL encoding effort, from 1 (fastest) to 10.
    int jpegxl_effort = 3;
};

// How `write` treats an existing file and missing parent directories, and how it encodes.
struct WriteOptions {
    WriteMode mode = default_write_mode;
    bool make_dirs = default_make_dirs;
    EncodeOptions encoding {};
};

struct CorruptData : alp::MessageCode<"the data is not a valid image"> {};

using RGB8 = radix::Raster<glm::u8vec3>;
using RGBA8 = radix::Raster<glm::u8vec4>;

// An 8-bit RGB or RGBA raster.
template <typename Image>
concept ImageRaster = std::same_as<Image, RGB8> || std::same_as<Image, RGBA8>;

// How `decode` checks the image.
struct DecodeOptions {
    // The size the image must have, if given.
    std::optional<glm::uvec2> expected_size = std::nullopt;
};
using ReadOptions = DecodeOptions;

struct Unknown : alp::MessageCode<"the image library reported a failure of unknown meaning"> {};

using EncodeError = alp::Error<Unsupported, OutOfMemory, Unknown>;

// Encodes `image`, with its first row at the top, in `format`, keeping all of its channels; JPEG XL tags the colour channels as sRGB.
// Fails if `image` is empty or `format` cannot store its channels.
// Each field of `options` must lie in its stated range.
template <ImageRaster Image>
[[nodiscard]]
alp::Result<bytes::vector, EncodeError> encode(const Image &image, Format format, EncodeOptions options = {});

using DecodeError = alp::Error<CorruptData, AllocError, Unsupported, Unknown>;

// Decodes the 8-bit image that `bytes` encodes, with its first row at the top and EXIF orientation ignored, dropping alpha or adding opaque alpha as `Image` needs.
// Fails if the image's size differs from `options.expected_size`.
template <ImageRaster Image>
[[nodiscard]]
alp::Result<Image, DecodeError> decode(bytes::span bytes, DecodeOptions options = {});

using ReadError = alp::Error<bytes::ReadError, DecodeError>;

// Reads and decodes the image file at `path`.
template <ImageRaster Image>
[[nodiscard]]
alp::Result<Image, ReadError> read(const std::filesystem::path &path, ReadOptions options = {});

using WriteError = alp::Error<Unsupported, EncodeError, bytes::WriteError>;

// Encodes `image` in the format that the extension of `path` names and writes it to the file at `path`.
template <ImageRaster Image>
[[nodiscard]]
alp::Result<void, WriteError> write(const Image &image, const std::filesystem::path &path, WriteOptions options = {});

} // namespace io::image
