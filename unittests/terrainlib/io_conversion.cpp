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

#include "../catch2_helpers.h"
#include "../result_helpers.h"

#include "io/conversion.h"

#include <array>
#include <cstdint>

namespace {

template <typename Pixel>
void check_round_trip(const int cv_type)
{
    cv::Mat source(2, 3, cv_type);
    auto bytes = std::span<uint8_t>(source.ptr<uint8_t>(), source.total() * source.elemSize());
    for (size_t index = 0; index < bytes.size(); ++index) {
        bytes[index] = static_cast<uint8_t>(index + 1);
    }

    const auto raster = REQUIRE_VALUE(io::conversion::to_raster<Pixel>(source));
    CHECK(raster.size() == glm::uvec2(3, 2));

    const cv::Mat result = REQUIRE_VALUE(io::conversion::to_mat(raster));
    CHECK(result.type() == source.type());
    CHECK(result.rows == source.rows);
    CHECK(result.cols == source.cols);
    CHECK(std::equal(bytes.begin(), bytes.end(), result.ptr<uint8_t>()));
}

} // namespace

TEST_CASE("io::conversion round trips supported scalar and vector pixels", "[io][conversion]")
{
    check_round_trip<uint8_t>(CV_8UC1);
    check_round_trip<int8_t>(CV_8SC1);
    check_round_trip<uint16_t>(CV_16UC1);
    check_round_trip<int16_t>(CV_16SC1);
    check_round_trip<int32_t>(CV_32SC1);
    check_round_trip<float>(CV_32FC1);
    check_round_trip<double>(CV_64FC1);
    check_round_trip<glm::u8vec2>(CV_8UC2);
    check_round_trip<glm::u8vec3>(CV_8UC3);
    check_round_trip<glm::u8vec4>(CV_8UC4);
    check_round_trip<glm::vec3>(CV_32FC3);
    check_round_trip<glm::dvec4>(CV_64FC4);
}

TEST_CASE("io::conversion copies non-contiguous matrices row by row", "[io][conversion]")
{
    cv::Mat backing(4, 6, CV_8UC3);
    for (int row = 0; row < backing.rows; ++row) {
        for (int column = 0; column < backing.cols; ++column) {
            backing.at<cv::Vec3b>(row, column)
                = cv::Vec3b(static_cast<uint8_t>(row), static_cast<uint8_t>(column), static_cast<uint8_t>(row * backing.cols + column));
        }
    }
    const cv::Mat source = backing(cv::Rect(1, 1, 3, 2));
    REQUIRE_FALSE(source.isContinuous());

    auto raster = REQUIRE_VALUE(io::conversion::to_raster<glm::u8vec3>(source));
    CHECK(raster.pixel({ 0, 0 }) == glm::u8vec3(1, 1, 7));
    CHECK(raster.pixel({ 2, 1 }) == glm::u8vec3(2, 3, 15));

    backing.setTo(cv::Scalar::all(0));
    CHECK(raster.pixel({ 0, 0 }) == glm::u8vec3(1, 1, 7));

    const auto round_trip = REQUIRE_VALUE(io::conversion::to_mat(raster));
    raster.fill(glm::u8vec3(0));
    CHECK(round_trip.at<cv::Vec3b>(0, 0) == cv::Vec3b(1, 1, 7));
}

TEST_CASE("io::conversion reports invalid inputs", "[io][conversion]")
{
    SECTION("multidimensional matrix")
    {
        const std::array sizes { 2, 2, 2 };
        const cv::Mat source(static_cast<int>(sizes.size()), sizes.data(), CV_8UC1);
        CHECK(REQUIRE_ERROR(io::conversion::to_raster<uint8_t>(source)).holds<Unsupported>());
    }

    SECTION("pixel type mismatch")
    {
        const cv::Mat source(2, 2, CV_8UC3);
        CHECK(REQUIRE_ERROR(io::conversion::to_raster<uint8_t>(source)).holds<io::conversion::TypeMismatch>());
    }

    SECTION("unsupported pixel")
    {
        struct Pixel {
            uint64_t value;
        };
        const cv::Mat source(2, 2, CV_8UC1);
        CHECK(REQUIRE_ERROR(io::conversion::to_raster<Pixel>(source)).holds<Unsupported>());
    }
}
