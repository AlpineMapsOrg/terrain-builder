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

#include <filesystem>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "common_error_codes.h"
#include "error/Error.h"
#include "error/Result.h"
#include "io/OpenCvError.h"
#include "io/alloc.h"
#include "io/bytes.h"
#include "io/image.h"
#include "io/image/Format.h"
#include "io/limits.h"

namespace io::mat {

using CorruptData = image::CorruptData;

// How `decode` lays out the image.
struct DecodeOptions {
    // The channels and depth OpenCV converts the image to.
    cv::ImreadModes mode = cv::IMREAD_COLOR;
};

using DecodeError = alp::Error<CorruptData, AllocError, OpenCvError>;
// Decodes the image that `bytes` encodes, with its colour channels in BGR order.
// Fails if `bytes` is larger than `max_buffer_size`.
[[nodiscard]]
alp::Result<cv::Mat, DecodeError> decode(bytes::span bytes, DecodeOptions options = {});

using ReadOptions = DecodeOptions;

using ReadError = alp::Error<bytes::ReadError, DecodeError>;
// Reads and decodes the image file at `path`.
[[nodiscard]]
alp::Result<cv::Mat, ReadError> read(const std::filesystem::path &path, ReadOptions options = {});

using EncodeOptions = image::EncodeOptions;

using EncodeError = alp::Error<Unsupported, OutOfMemory, OpenCvError>;
// Encodes `image`, whose colour channels are in BGR order, in `format`.
// Fails if `image` is empty or `format` cannot store it.
// Each field of `options` must lie in its stated range.
[[nodiscard]]
alp::Result<bytes::vector, EncodeError> encode(const cv::Mat &image, image::Format format, EncodeOptions options = {});

using WriteOptions = image::WriteOptions;

using WriteError = alp::Error<Unsupported, EncodeError, bytes::WriteError>;
// Encodes `image` in the format that the extension of `path` names and writes it to the file at `path`.
[[nodiscard]]
alp::Result<void, WriteError> write(const cv::Mat &image, const std::filesystem::path &path, WriteOptions options = {});

} // namespace io::mat
