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

#include "io/mat.h"

#include <cstdint>
#include <limits>
#include <new>
#include <string>
#include <vector>

#include <libassert/assert.hpp>
#include <opencv2/imgcodecs.hpp>

#include "error/try.h"
#include "error/utils.h"
#include "fmt_impls.h"

namespace io::mat {

alp::Result<cv::Mat, DecodeError> decode(const bytes::span bytes, const DecodeOptions options) {
    constexpr auto context = "decoding an image";

    if (bytes.empty()) {
        return alp::make_error(CorruptData{}, "the data is empty").context(context);
    }
    if (bytes.size() > max_buffer_size) {
        return alp::make_error(BufferLimitExceeded(max_buffer_size, bytes.size()), context);
    }
    static_assert(max_buffer_size <= static_cast<size_t>(std::numeric_limits<int>::max()), "OpenCV sizes are int");
    try {
        cv::Mat image = cv::imdecode(cv::_InputArray(bytes.data(), static_cast<int>(bytes.size())), options.mode);
        if (image.empty()) {
            return alp::make_error(CorruptData{}, context);
        }
        return image;
    } catch (const std::bad_alloc &) {
        return alp::make_error(OutOfMemory{}, context);
    } catch (const cv::Exception &exception) {
        if (exception.code == cv::Error::StsNoMem) {
            return alp::make_error(OutOfMemory{}, context, OpenCvError(exception));
        }
        return alp::make_error(OpenCvError(exception), context);
    }
}

alp::Result<cv::Mat, ReadError> read(const std::filesystem::path &path, const ReadOptions options) {
    const auto context = alp::make_context("reading an image from {}", path);
    ALP_TRY_CONTEXT(const bytes::vector bytes, bytes::read(path), context);
    ALP_TRY_CONTEXT(cv::Mat image, mat::decode(bytes, options), context);
    return image;
}

alp::Result<bytes::vector, EncodeError> encode(const cv::Mat &image, const image::Format format, const EncodeOptions options) {
    if (image.empty()) {
        return alp::make_error(Unsupported{}, "an image without pixels has no encoding");
    }
    ASSERT(options.jpeg_quality >= 0 && options.jpeg_quality <= 100 && options.png_compression >= 0 && options.png_compression <= 9);
    ASSERT(options.jpegxl_distance >= 0 && options.jpegxl_distance <= 25 && options.jpegxl_effort >= 1 && options.jpegxl_effort <= 10);
    const auto context = alp::make_context("encoding an image as {}", image::to_extension(format));

    std::vector<int> parameters;
    switch (format) {
    case image::Format::Jpeg:
        if (image.channels() == 4) {
            return alp::make_error(Unsupported{}, "JPEG cannot store an alpha channel");
        }
        parameters = {cv::IMWRITE_JPEG_QUALITY, options.jpeg_quality};
        break;
    case image::Format::Png:
        parameters = {cv::IMWRITE_PNG_COMPRESSION, options.png_compression};
        break;
    case image::Format::JpegXl:
        return alp::make_error(Unsupported{}, "an OpenCV matrix cannot be encoded as JPEG XL yet");
    }

    bytes::vector encoded;
    try {
        if (!cv::imencode(std::string(image::to_extension(format)), image, encoded, parameters)) {
            return alp::make_error(OpenCvError(), context);
        }
    } catch (const std::bad_alloc &) {
        return alp::make_error(OutOfMemory{}, context);
    } catch (const cv::Exception &exception) {
        if (exception.code == cv::Error::StsNoMem) {
            return alp::make_error(OutOfMemory{}, context, OpenCvError(exception));
        }
        return alp::make_error(OpenCvError(exception), context);
    }
    return encoded;
}

alp::Result<void, WriteError> write(const cv::Mat &image, const std::filesystem::path &path, const WriteOptions options) {
    const auto context = alp::make_context("writing an image to {}", path);
    ALP_TRY_CONTEXT(const image::Format format, image::format_from_path(path), context);
    ALP_TRY_CONTEXT(const bytes::vector bytes, mat::encode(image, format, options.encoding), context);
    ALP_TRY_CONTEXT(bytes::write(bytes, path, {.mode = options.mode, .make_dirs = options.make_dirs}), context);
    return {};
}

} // namespace io::mat
