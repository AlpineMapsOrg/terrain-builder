#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <filesystem>
#include <numeric>
#include <span>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "../opencv_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "error/prelude.h"
#include "io/FileError.h"
#include "io/bytes.h"
#include "texture/io.h"

namespace {

// An image of `type` in which no two bytes are equal.
cv::Mat make_image(const int type) {
    cv::Mat image(3, 4, type);
    const std::span<uint8_t> bytes(image.data, image.total() * image.elemSize());
    const uint8_t first = 0;
    std::iota(bytes.begin(), bytes.end(), first);
    return image;
}

} // namespace

TEST_CASE("Encoding and decoding an image in a lossless format round-trips its pixels", "[texture][io]") {
    const cv::Mat image = make_image(CV_8UC3);

    const std::vector<uint8_t> bytes = REQUIRE_VALUE(texture::io::encode_to_bytes(image, ".png"));
    const cv::Mat decoded = REQUIRE_VALUE(texture::io::decode_bytes(bytes));

    CHECK(mat_equals(decoded, image));
}

TEST_CASE("Decoding as unchanged keeps the alpha channel that decoding as colour drops", "[texture][io]") {
    const cv::Mat image = make_image(CV_8UC4);
    const std::vector<uint8_t> bytes = REQUIRE_VALUE(texture::io::encode_to_bytes(image, ".png"));

    const cv::Mat colour = REQUIRE_VALUE(texture::io::decode_bytes(bytes));
    const cv::Mat unchanged = REQUIRE_VALUE(texture::io::decode_bytes(bytes, cv::IMREAD_UNCHANGED));

    CHECK(colour.channels() == 3);
    CHECK(mat_equals(unchanged, image));
}

TEST_CASE("Decoding bytes that are not an image fails with a decode error", "[texture][io]") {
    const std::vector<uint8_t> bytes = {1, 2, 3, 4};

    const alp::Error<texture::io::DecodeError> error = REQUIRE_ERROR(texture::io::decode_bytes(bytes));

    CHECK(error.holds<texture::io::DecodeError>());
}

TEST_CASE("Decoding no bytes fails with a decode error", "[texture][io]") {
    const alp::Error<texture::io::DecodeError> error = REQUIRE_ERROR(texture::io::decode_bytes(std::span<const uint8_t>()));

    CHECK(error.holds<texture::io::DecodeError>());
}

TEST_CASE("Encoding for an extension that names no image format fails, naming the extension", "[texture][io]") {
    const alp::Error<io::UnsupportedFormat, texture::io::EncodeError> error = REQUIRE_ERROR(texture::io::encode_to_bytes(make_image(CV_8UC3), ".not_an_image"));

    CHECK(error.holds_code(io::UnsupportedFormat{".not_an_image"}));
}

TEST_CASE("Encoding an image whose channel layout the format cannot hold fails with an encode error", "[texture][io]") {
    const alp::Error<io::UnsupportedFormat, texture::io::EncodeError> error = REQUIRE_ERROR(texture::io::encode_to_bytes(make_image(CV_8UC2), ".png"));

    CHECK(error.holds<texture::io::EncodeError>());
}

TEST_CASE("Writing and reading an image file in a lossless format round-trips its pixels", "[texture][io]") {
    const TempDirectory directory("texture_io_round_trip");
    const std::filesystem::path path = directory.path() / "image.png";
    const cv::Mat image = make_image(CV_8UC3);

    REQUIRE_VALUE(texture::io::write_to_path(image, path));
    const cv::Mat read = REQUIRE_VALUE(texture::io::read_from_path(path));

    CHECK(mat_equals(read, image));
}

TEST_CASE("Reading a missing image file reports that it does not exist, naming the path", "[texture][io]") {
    const TempDirectory directory("texture_io_missing");
    const std::filesystem::path path = directory.path() / "missing.png";

    const texture::io::ReadError error = REQUIRE_ERROR(texture::io::read_from_path(path));

    CHECK(error.holds_code(io::make_file_not_found(path)));
}

TEST_CASE("Reading a file that is not an image fails with a decode error", "[texture][io]") {
    const TempDirectory directory("texture_io_not_an_image");
    const std::filesystem::path path = directory.path() / "broken.png";
    REQUIRE_VALUE(io::write_bytes_to_path(std::vector<uint8_t>{1, 2, 3, 4}, path));

    const texture::io::ReadError error = REQUIRE_ERROR(texture::io::read_from_path(path));

    CHECK(error.holds<texture::io::DecodeError>());
}

TEST_CASE("Writing an image to a path whose extension names no image format fails, naming the extension, and creates no file", "[texture][io]") {
    const TempDirectory directory("texture_io_unknown_extension");
    const std::filesystem::path path = directory.path() / "image.not_an_image";

    const texture::io::WriteError error = REQUIRE_ERROR(texture::io::write_to_path(make_image(CV_8UC3), path));

    CHECK(error.holds_code(io::UnsupportedFormat{".not_an_image"}));
    CHECK(!std::filesystem::exists(path));
}

TEST_CASE("Writing an image below a file fails with a file error", "[texture][io]") {
    const TempDirectory directory("texture_io_below_file");
    const std::filesystem::path blocker = directory.path() / "blocker";
    REQUIRE_VALUE(io::write_bytes_to_path(std::vector<uint8_t>{1}, blocker));

    const texture::io::WriteError error = REQUIRE_ERROR(texture::io::write_to_path(make_image(CV_8UC3), blocker / "image.png"));

    CHECK(error.holds<io::FileError>());
}
