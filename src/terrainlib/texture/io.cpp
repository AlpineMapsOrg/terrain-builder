#include <string>

#include <libassert/assert.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "error/Context.h"
#include "error/try.h"
#include "error/utils.h"
#include "fmt_impls.h"
#include "io/bytes.h"
#include "texture/io.h"

namespace texture::io {

alp::Result<cv::Mat, DecodeError> decode_bytes(const std::span<const uint8_t> bytes, const cv::ImreadModes mode) {
    if (bytes.empty()) {
        return alp::make_error(DecodeError{});
    }

    const cv::Mat encoded(1, static_cast<int>(bytes.size()), CV_8UC1, const_cast<uint8_t *>(bytes.data()));
    cv::Mat image = cv::imdecode(encoded, mode);
    if (image.empty()) {
        return alp::make_error(DecodeError{});
    }
    return image;
}

alp::Result<std::vector<uint8_t>, ::io::UnsupportedFormat, EncodeError> encode_to_bytes(const cv::Mat &image, const std::string_view extension_with_dot) {
    ASSERT(!image.empty());
    const std::string extension(extension_with_dot);
    if (!cv::haveImageWriter(extension)) {
        return alp::make_error(::io::UnsupportedFormat{.extension = extension});
    }

    std::vector<uint8_t> bytes;
    try {
        if (!cv::imencode(extension, image, bytes)) {
            // OpenCV reports no reason when it fails without throwing.
            return alp::make_error(EncodeError{.reason = "OpenCV gave no reason"});
        }
    } catch (const cv::Exception &exception) {
        return alp::make_error(EncodeError{.reason = exception.err});
    }
    return bytes;
}

alp::Result<cv::Mat, ReadError> read_from_path(const std::filesystem::path &path, const cv::ImreadModes mode) {
    const auto context = alp::make_context("reading an image from {}", path);
    ALP_TRY_CONTEXT(const std::vector<uint8_t> &bytes, ::io::read_bytes_from_path(path), context);
    ALP_TRY_CONTEXT(cv::Mat image, decode_bytes(bytes, mode), context);
    return image;
}

alp::Result<void, WriteError> write_to_path(const cv::Mat &image, const std::filesystem::path &path) {
    const auto context = alp::make_context("writing an image to {}", path);
    ALP_TRY_CONTEXT(const std::vector<uint8_t> &bytes, encode_to_bytes(image, path.extension().string()), context);
    ALP_TRY_CONTEXT(::io::write_bytes_to_path(bytes, path), context);
    return {};
}

} // namespace texture::io
