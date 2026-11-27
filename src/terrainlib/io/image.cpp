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

#include "io/image.h"

#include <cstdint>
#include <new>
#include <optional>
#include <ranges>
#include <string>
#include <utility>

#include <fmt/format.h>
#include <jxl/decode.h>
#include <jxl/decode_cxx.h>
#include <jxl/encode.h>
#include <jxl/encode_cxx.h>
#include <libassert/assert.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "error/try.h"
#include "error/utils.h"
#include "fmt_impls.h"
#include "io/OpenCvError.h"
#include "io/conversion.h"
#include "io/image/JxlError.h"
#include "io/mat.h"

namespace io::image {
namespace {

    // The error for a failure on the OpenCV path, with a failure OpenCV reported as Unknown and the OpenCvError as its source.
    EncodeError to_image_error(mat::EncodeError error) {
        return std::move(error).map<OpenCvError>(Unknown{});
    }

    // The error for a failure on the OpenCV path, with a failure OpenCV reported as Unknown and the OpenCvError as its source.
    DecodeError to_image_error(mat::DecodeError error) {
        return std::move(error).map<OpenCvError>(Unknown{});
    }

    template <typename Pixel>
    alp::Result<bytes::vector, EncodeError> encode_opencv(const radix::Raster<Pixel> &image, const Format format, const EncodeOptions options) {
        const auto context = alp::make_context("encoding an image as {}", format);
        auto mat_result = conversion::to_mat(image);
        if (!mat_result.has_value()) {
            return to_image_error(std::move(mat_result).error()).context(context);
        }
        cv::Mat mat = std::move(mat_result).value();
        try {
            cv::cvtColor(mat, mat, Pixel::length() == 4 ? cv::COLOR_RGBA2BGRA : cv::COLOR_RGB2BGR);
        } catch (const std::bad_alloc &) {
            return alp::make_error(OutOfMemory{}, context);
        } catch (const cv::Exception &exception) {
            if (exception.code == cv::Error::StsNoMem) {
                return alp::make_error(OutOfMemory{}, context, OpenCvError(exception));
            }
            return alp::make_error(Unknown{}, context, OpenCvError(exception));
        }
        auto encoded_result = mat::encode(mat, format, options);
        if (!encoded_result.has_value()) {
            return to_image_error(std::move(encoded_result).error());
        }
        return std::move(encoded_result).value();
    }

    template <typename Pixel>
    alp::Result<bytes::vector, EncodeError> encode_jpegxl(const radix::Raster<Pixel> &image, const EncodeOptions options)
    try {
        ASSERT(options.jpegxl_distance >= 0 && options.jpegxl_distance <= 25 && options.jpegxl_effort >= 1 && options.jpegxl_effort <= 10);
        constexpr auto context = "encoding an image as JPEG XL";
        const auto encoder = JxlEncoderMake(nullptr);
        if (!encoder) {
            return alp::make_error(OutOfMemory{}, "creating the JPEG XL encoder");
        }
        const auto failed = [&encoder, context]() -> EncodeError {
            const JxlError error = JxlError::get_last_error(*encoder);
            switch (error.code()) {
            case JXL_ENC_ERR_OOM:
                return alp::make_error(OutOfMemory{}, context);
            case JXL_ENC_ERR_NOT_SUPPORTED:
                return alp::make_error(Unsupported{}, context);
            case JXL_ENC_ERR_API_USAGE:
                PANIC("libjxl reported a misuse of its encoder API");
            default:
                return alp::make_error(Unknown{}, context, error);
            }
        };
        const bool lossless = options.jpegxl_distance == 0;
        JxlBasicInfo info;
        JxlEncoderInitBasicInfo(&info);
        info.xsize = image.width();
        info.ysize = image.height();
        info.bits_per_sample = 8;
        info.num_color_channels = 3;
        info.alpha_bits = Pixel::length() == 4 ? 8 : 0;
        info.num_extra_channels = Pixel::length() == 4 ? 1 : 0;
        // Lossy coding converts to XYB; lossless coding must keep the original colour space.
        info.uses_original_profile = lossless ? JXL_TRUE : JXL_FALSE;
        JxlColorEncoding colour_encoding;
        JxlColorEncodingSetToSRGB(&colour_encoding, JXL_FALSE);
        if (JxlEncoderSetBasicInfo(encoder.get(), &info) != JXL_ENC_SUCCESS || JxlEncoderSetColorEncoding(encoder.get(), &colour_encoding) != JXL_ENC_SUCCESS) {
            return failed();
        }
        auto *settings = JxlEncoderFrameSettingsCreate(encoder.get(), nullptr);
        const JxlPixelFormat pixel_format{Pixel::length(), JXL_TYPE_UINT8, JXL_NATIVE_ENDIAN, 0};
        const auto pixels = image.bytes();
        if (!settings || JxlEncoderFrameSettingsSetOption(settings, JXL_ENC_FRAME_SETTING_EFFORT, options.jpegxl_effort) != JXL_ENC_SUCCESS
            || JxlEncoderSetFrameDistance(settings, options.jpegxl_distance) != JXL_ENC_SUCCESS
            // Alpha uses the colour distance; libjxl would otherwise keep it lossless.
            || (Pixel::length() == 4 && JxlEncoderSetExtraChannelDistance(settings, 0, -1.0f) != JXL_ENC_SUCCESS)
            || (lossless && JxlEncoderSetFrameLossless(settings, JXL_TRUE) != JXL_ENC_SUCCESS)
            || JxlEncoderAddImageFrame(settings, &pixel_format, pixels.data(), pixels.size()) != JXL_ENC_SUCCESS) {
            return failed();
        }
        JxlEncoderCloseInput(encoder.get());
        bytes::vector encoded(pixels.size() / 8 + 4096);
        size_t used = 0;
        JxlEncoderStatus status = JXL_ENC_NEED_MORE_OUTPUT;
        while (status == JXL_ENC_NEED_MORE_OUTPUT) {
            if (used == encoded.size()) {
                encoded.resize(2 * encoded.size());
            }
            uint8_t *next = encoded.data() + used;
            size_t available = encoded.size() - used;
            status = JxlEncoderProcessOutput(encoder.get(), &next, &available);
            used = encoded.size() - available;
        }
        if (status != JXL_ENC_SUCCESS) {
            return failed();
        }
        encoded.resize(used);
        return encoded;
    } catch (const std::bad_alloc &) {
        return alp::make_error(OutOfMemory{}, "encoding an image as JPEG XL");
    }

    template <typename Pixel>
    alp::Result<radix::Raster<Pixel>, DecodeError> decode_opencv(const bytes::span bytes, const DecodeOptions options) {
        // Unchanged, so that the depth and channel count of the file are visible.
        auto decoded_result = mat::decode(bytes, {.mode = cv::IMREAD_UNCHANGED});
        if (!decoded_result.has_value()) {
            return to_image_error(std::move(decoded_result).error());
        }
        cv::Mat decoded = std::move(decoded_result).value();
        const glm::uvec2 size(decoded.cols, decoded.rows);
        if (options.expected_size && size != *options.expected_size) {
            return alp::make_error(CorruptData{}, fmt::format("the image is {}x{}, not the expected {}x{}", size.x, size.y, options.expected_size->x, options.expected_size->y));
        }
        if (decoded.depth() != CV_8U) {
            return alp::make_error(Unsupported{}, fmt::format("the image has {} samples, not 8-bit ones", cv::depthToString(decoded.depth())));
        }
        constexpr bool alpha = Pixel::length() == 4;
        int colour_conversion = 0;
        switch (decoded.channels()) {
        case 1:
            colour_conversion = alpha ? cv::COLOR_GRAY2RGBA : cv::COLOR_GRAY2RGB;
            break;
        case 3:
            colour_conversion = alpha ? cv::COLOR_BGR2RGBA : cv::COLOR_BGR2RGB;
            break;
        case 4:
            colour_conversion = alpha ? cv::COLOR_BGRA2RGBA : cv::COLOR_BGRA2RGB;
            break;
        default:
            return alp::make_error(Unsupported{}, fmt::format("the image has {} channels", decoded.channels()));
        }
        constexpr auto context = "converting the decoded image";
        try {
            cv::cvtColor(decoded, decoded, colour_conversion);
        } catch (const std::bad_alloc &) {
            return alp::make_error(OutOfMemory{}, context);
        } catch (const cv::Exception &exception) {
            if (exception.code == cv::Error::StsNoMem) {
                return alp::make_error(OutOfMemory{}, context, OpenCvError(exception));
            }
            return alp::make_error(Unknown{}, context, OpenCvError(exception));
        }

        auto raster_result = conversion::to_raster<Pixel>(decoded);
        if (!raster_result.has_value()) {
            conversion::ToRasterError error = std::move(raster_result).error();
            auto split_result = std::move(error).split<conversion::TypeMismatch>();
            ASSERT(!split_result.has_value(), "cvtColor produced a matrix of another type than the raster's");
            return to_image_error(std::move(split_result).error());
        }
        return std::move(raster_result).value();
    }

    // Reads the first frame. libjxl drops or adds opaque alpha and expands grey to RGB.
    template <typename Pixel>
    alp::Result<radix::Raster<Pixel>, DecodeError> decode_jpegxl(const bytes::span bytes, const DecodeOptions options)
    try {
        if (bytes.size() > max_buffer_size) {
            return alp::make_error(BufferLimitExceeded(max_buffer_size, bytes.size()));
        }
        const auto decoder = JxlDecoderMake(nullptr);
        if (!decoder) {
            return alp::make_error(OutOfMemory{}, "creating the JPEG XL decoder");
        }
        // Keep the stored row order, as for EXIF orientation.
        if (JxlDecoderSubscribeEvents(decoder.get(), JXL_DEC_BASIC_INFO | JXL_DEC_FULL_IMAGE) != JXL_DEC_SUCCESS
            || JxlDecoderSetKeepOrientation(decoder.get(), JXL_TRUE) != JXL_DEC_SUCCESS
            || JxlDecoderSetInput(decoder.get(), bytes.data(), bytes.size()) != JXL_DEC_SUCCESS) {
            PANIC("libjxl rejected the decoder setup");
        }
        JxlDecoderCloseInput(decoder.get());
        const JxlPixelFormat pixel_format{Pixel::length(), JXL_TYPE_UINT8, JXL_NATIVE_ENDIAN, 0};
        std::optional<radix::Raster<Pixel>> raster;
        while (true) {
            switch (JxlDecoderProcessInput(decoder.get())) {
            case JXL_DEC_BASIC_INFO: {
                JxlBasicInfo info;
                if (JxlDecoderGetBasicInfo(decoder.get(), &info) != JXL_DEC_SUCCESS) {
                    return alp::make_error(CorruptData{}, "reading the JPEG XL basic info");
                }
                const glm::uvec2 size(info.xsize, info.ysize);
                if (options.expected_size && size != *options.expected_size) {
                    return alp::make_error(CorruptData{}, "JPEG XL image size differs from the expected size");
                }
                raster.emplace(size);
                break;
            }
            case JXL_DEC_NEED_IMAGE_OUT_BUFFER: {
                if (!raster) {
                    return alp::make_error(CorruptData{}, "JPEG XL image data precedes its basic info");
                }
                const auto pixels = raster->bytes();
                if (JxlDecoderSetImageOutBuffer(decoder.get(), &pixel_format, pixels.data(), pixels.size()) != JXL_DEC_SUCCESS) {
                    return alp::make_error(CorruptData{}, "JPEG XL image does not fit its basic info");
                }
                break;
            }
            case JXL_DEC_FULL_IMAGE:
                return std::move(*raster);
            case JXL_DEC_NEED_MORE_INPUT:
                return alp::make_error(CorruptData{}, "truncated JPEG XL image");
            case JXL_DEC_SUCCESS:
                return alp::make_error(CorruptData{}, "JPEG XL image has no frame");
            default:
                return alp::make_error(CorruptData{}, "JPEG XL decoding failed");
            }
        }
    } catch (const std::bad_alloc &) {
        return alp::make_error(OutOfMemory{}, "decoding a JPEG XL image");
    }

} // namespace

template <ImageRaster Image>
alp::Result<bytes::vector, EncodeError> encode(const Image &image, const Format format, const EncodeOptions options) {
    if (image.width() == 0 || image.height() == 0) {
        return alp::make_error(Unsupported{}, "an image without pixels has no encoding");
    }
    if (format == Format::JpegXl) {
        return encode_jpegxl(image, options);
    }
    return encode_opencv(image, format, options);
}

template <ImageRaster Image>
alp::Result<Image, DecodeError> decode(const bytes::span bytes, const DecodeOptions options) {
    using Pixel = std::ranges::range_value_t<Image>;
    const JxlSignature signature = JxlSignatureCheck(bytes.data(), bytes.size());
    if (signature == JXL_SIG_CODESTREAM || signature == JXL_SIG_CONTAINER) {
        return decode_jpegxl<Pixel>(bytes, options);
    }
    return decode_opencv<Pixel>(bytes, options);
}

template <ImageRaster Image>
alp::Result<Image, ReadError> read(const std::filesystem::path &path, const ReadOptions options) {
    const auto context = alp::make_context("reading an image from {}", path);
    ALP_TRY_CONTEXT(const bytes::vector bytes, bytes::read(path), context);
    ALP_TRY_CONTEXT(Image image, image::decode<Image>(bytes, options), context);
    return image;
}

template <ImageRaster Image>
alp::Result<void, WriteError> write(const Image &image, const std::filesystem::path &path, const WriteOptions options) {
    const auto context = alp::make_context("writing an image to {}", path);
    ALP_TRY_CONTEXT(const Format format, format_from_path(path), context);
    ALP_TRY_CONTEXT(const bytes::vector bytes, image::encode(image, format, options.encoding), context);
    ALP_TRY_CONTEXT(bytes::write(bytes, path, {.mode = options.mode, .make_dirs = options.make_dirs}), context);
    return {};
}

template alp::Result<bytes::vector, EncodeError> encode<RGB8>(const RGB8 &, Format, EncodeOptions);
template alp::Result<bytes::vector, EncodeError> encode<RGBA8>(const RGBA8 &, Format, EncodeOptions);
template alp::Result<RGB8, DecodeError> decode<RGB8>(bytes::span, DecodeOptions);
template alp::Result<RGBA8, DecodeError> decode<RGBA8>(bytes::span, DecodeOptions);
template alp::Result<RGB8, ReadError> read<RGB8>(const std::filesystem::path &, ReadOptions);
template alp::Result<RGBA8, ReadError> read<RGBA8>(const std::filesystem::path &, ReadOptions);
template alp::Result<void, WriteError> write<RGB8>(const RGB8 &, const std::filesystem::path &, WriteOptions);
template alp::Result<void, WriteError> write<RGBA8>(const RGBA8 &, const std::filesystem::path &, WriteOptions);

} // namespace io::image
