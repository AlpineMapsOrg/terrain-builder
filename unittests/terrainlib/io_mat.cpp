/*****************************************************************************
 * AlpineMaps.org
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

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

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
#include "io/filesystem.h"
#include "io/image/Format.h"
#include "io/mat.h"

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

TEST_CASE("Encoding and decoding an image in a lossless format round-trips its pixels", "[io][mat]") {
    const cv::Mat image = make_image(CV_8UC3);

    const std::vector<uint8_t> bytes = REQUIRE_VALUE(io::mat::encode(image, io::image::Format::Png));
    const cv::Mat decoded = REQUIRE_VALUE(io::mat::decode(bytes));

    CHECK(mat_equals(decoded, image));
}

TEST_CASE("Encoding an empty image fails as unsupported", "[io][mat]") {
    const io::image::Format format = GENERATE(io::image::Format::Jpeg, io::image::Format::Png);

    const auto error = REQUIRE_ERROR(io::mat::encode(cv::Mat(), format));

    CHECK(error.holds<Unsupported>());
}

TEST_CASE("Decoding as unchanged keeps the alpha channel that decoding as colour drops", "[io][mat]") {
    const cv::Mat image = make_image(CV_8UC4);
    const std::vector<uint8_t> bytes = REQUIRE_VALUE(io::mat::encode(image, io::image::Format::Png));

    const cv::Mat colour = REQUIRE_VALUE(io::mat::decode(bytes));
    const cv::Mat unchanged = REQUIRE_VALUE(io::mat::decode(bytes, {.mode = cv::IMREAD_UNCHANGED}));

    CHECK(colour.channels() == 3);
    CHECK(mat_equals(unchanged, image));
}

TEST_CASE("Decoding bytes that are not an image fails as corrupt data", "[io][mat]") {
    const std::vector<uint8_t> bytes = {1, 2, 3, 4};

    const auto error = REQUIRE_ERROR(io::mat::decode(bytes));

    CHECK(error.holds<io::mat::CorruptData>());
}

TEST_CASE("Decoding no bytes fails as corrupt data", "[io][mat]") {
    const auto error = REQUIRE_ERROR(io::mat::decode(std::span<const uint8_t>()));

    CHECK(error.holds<io::mat::CorruptData>());
}

TEST_CASE("Writing and reading an image file in a lossless format round-trips its pixels", "[io][mat]") {
    const TempDirectory directory("io_mat_round_trip");
    const std::filesystem::path path = directory.path() / "image.png";
    const cv::Mat image = make_image(CV_8UC3);

    REQUIRE_VALUE(io::mat::write(image, path));
    const cv::Mat read = REQUIRE_VALUE(io::mat::read(path));

    CHECK(mat_equals(read, image));
}

TEST_CASE("Reading a missing image file reports that it does not exist, naming the path", "[io][mat]") {
    const TempDirectory directory("io_mat_missing");
    const std::filesystem::path path = directory.path() / "missing.png";

    const auto error = REQUIRE_ERROR(io::mat::read(path));

    CHECK(error.holds_code(io::make_file_not_found(path)));
}

TEST_CASE("Reading a file that is not an image fails as corrupt data", "[io][mat]") {
    const TempDirectory directory("io_mat_not_an_image");
    const std::filesystem::path path = directory.path() / "broken.png";
    REQUIRE_VALUE(io::bytes::write(std::vector<uint8_t>{1, 2, 3, 4}, path));

    const auto error = REQUIRE_ERROR(io::mat::read(path));

    CHECK(error.holds<io::mat::CorruptData>());
}

TEST_CASE("Writing an image with an unknown extension fails as unsupported", "[io][mat]") {
    const TempDirectory directory("io_mat_unknown_extension");
    const std::filesystem::path path = directory.path() / "image.not_an_image";

    const auto error = REQUIRE_ERROR(io::mat::write(make_image(CV_8UC3), path));

    CHECK(error.holds<Unsupported>());
    CHECK(!REQUIRE_VALUE(io::filesystem::exists(path)));
}

TEST_CASE("Writing an image below a file fails with a file error", "[io][mat]") {
    const TempDirectory directory("io_mat_below_file");
    const std::filesystem::path blocker = directory.path() / "blocker";
    REQUIRE_VALUE(io::bytes::write(std::vector<uint8_t>{1}, blocker));

    const auto error = REQUIRE_ERROR(io::mat::write(make_image(CV_8UC3), blocker / "image.png"));

    CHECK(error.holds<io::FileError>());
}
