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

#include <cstdint>
#include <cstring>
#include <limits>
#include <new>
#include <type_traits>

#include <fmt/format.h>
#include <glm/glm.hpp>
#include <libassert/assert.hpp>
#include <opencv2/core.hpp>
#include <radix/raster.h>

#include "common_error_codes.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/utils.h"
#include "io/OpenCvError.h"
#include "type_name.h"

namespace io::conversion {

struct TypeMismatch : alp::MessageCode<"the matrix type is not the one the pixel type maps to"> {};

namespace detail {

    template <typename Channel>
    inline constexpr int cv_depth = -1;

    template <>
    inline constexpr int cv_depth<std::uint8_t> = CV_8U;
    template <>
    inline constexpr int cv_depth<std::int8_t> = CV_8S;
    template <>
    inline constexpr int cv_depth<std::uint16_t> = CV_16U;
    template <>
    inline constexpr int cv_depth<std::int16_t> = CV_16S;
    template <>
    inline constexpr int cv_depth<std::int32_t> = CV_32S;
    template <>
    inline constexpr int cv_depth<float> = CV_32F;
    template <>
    inline constexpr int cv_depth<double> = CV_64F;

    template <typename Pixel>
    struct PixelTraits {
        using channel_type = std::remove_cv_t<Pixel>;
        static constexpr int channels = 1;
        static constexpr bool supported = cv_depth<channel_type> >= 0;

        static channel_type& channel(Pixel& pixel, const int) { return pixel; }

        static const channel_type& channel(const Pixel& pixel, const int) { return pixel; }
    };

    template <glm::length_t Length, typename Channel, glm::qualifier Qualifier>
    struct PixelTraits<glm::vec<Length, Channel, Qualifier>> {
        using channel_type = Channel;
        static constexpr int channels = static_cast<int>(Length);
        static constexpr bool supported = channels >= 2 && channels <= 4 && cv_depth<channel_type> >= 0;

        static channel_type& channel(glm::vec<Length, Channel, Qualifier>& pixel, const int index) { return pixel[static_cast<glm::length_t>(index)]; }

        static const channel_type& channel(const glm::vec<Length, Channel, Qualifier>& pixel, const int index)
        {
            return pixel[static_cast<glm::length_t>(index)];
        }
    };

    template <typename Pixel>
    using Traits = PixelTraits<std::remove_cv_t<Pixel>>;

    template <typename Pixel>
    inline constexpr int cv_type = Traits<Pixel>::supported ? CV_MAKETYPE(cv_depth<typename Traits<Pixel>::channel_type>, Traits<Pixel>::channels) : -1;

    template <typename Pixel>
    inline constexpr bool has_packed_layout = Traits<Pixel>::supported
        && std::is_trivially_copyable_v<Pixel> && sizeof(Pixel) == sizeof(typename Traits<Pixel>::channel_type) * Traits<Pixel>::channels;

    template <typename Pixel>
    void copy_mat_to_raster(const cv::Mat& source, radix::Raster<Pixel>& destination)
    {
        using PixelInfo = Traits<Pixel>;
        using Channel = typename PixelInfo::channel_type;

        const auto row_bytes = static_cast<std::size_t>(source.cols) * sizeof(Pixel);
        if constexpr (has_packed_layout<Pixel>) {
            for (int row = 0; row < source.rows; ++row) {
                std::memcpy(destination.data() + static_cast<std::size_t>(row) * source.cols, source.ptr(row), row_bytes);
            }
        } else {
            for (int row = 0; row < source.rows; ++row) {
                const Channel* source_row = source.ptr<Channel>(row);
                for (int column = 0; column < source.cols; ++column) {
                    Pixel& pixel = destination.pixel({
                        static_cast<unsigned>(column),
                        static_cast<unsigned>(row),
                    });
                    for (int channel = 0; channel < PixelInfo::channels; ++channel) {
                        PixelInfo::channel(pixel, channel) = source_row[column * PixelInfo::channels + channel];
                    }
                }
            }
        }
    }

    template <typename Pixel>
    void copy_raster_to_mat(const radix::Raster<Pixel>& source, cv::Mat& destination)
    {
        using PixelInfo = Traits<Pixel>;
        using Channel = typename PixelInfo::channel_type;

        const auto row_bytes = static_cast<std::size_t>(destination.cols) * sizeof(Pixel);
        if constexpr (has_packed_layout<Pixel>) {
            for (int row = 0; row < destination.rows; ++row) {
                std::memcpy(destination.ptr(row), source.data() + static_cast<std::size_t>(row) * destination.cols, row_bytes);
            }
        } else {
            for (int row = 0; row < destination.rows; ++row) {
                Channel* destination_row = destination.ptr<Channel>(row);
                for (int column = 0; column < destination.cols; ++column) {
                    const Pixel& pixel = source.pixel({
                        static_cast<unsigned>(column),
                        static_cast<unsigned>(row),
                    });
                    for (int channel = 0; channel < PixelInfo::channels; ++channel) {
                        destination_row[column * PixelInfo::channels + channel] = PixelInfo::channel(pixel, channel);
                    }
                }
            }
        }
    }

} // namespace detail

using ToRasterError = alp::Error<Unsupported, TypeMismatch, OutOfMemory, OpenCvError>;

template <typename Pixel>
alp::Result<radix::Raster<Pixel>, ToRasterError> to_raster(const cv::Mat& source)
{
    if constexpr (!detail::Traits<Pixel>::supported) {
        return alp::make_error(Unsupported {}, fmt::format("there is no OpenCV type corresponding to {}", type_name<Pixel>()));
    } else {
        if (source.empty()) {
            return radix::Raster<Pixel>();
        }
        if (source.dims != 2) {
            return alp::make_error(Unsupported {}, fmt::format("the matrix has {} dimensions", source.dims));
        }
        if (source.type() != detail::cv_type<Pixel>) {
            return alp::make_error(TypeMismatch {},
                fmt::format("the matrix has type {}, {} corresponds to {}", cv::typeToString(source.type()), type_name<Pixel>(),
                    cv::typeToString(detail::cv_type<Pixel>)));
        }
        const glm::ivec2 size(source.cols, source.rows);
        ASSERT(glm::all(glm::greaterThanEqual(size, glm::ivec2(0))));

        constexpr auto context = "allocating the raster";
        try {
            radix::Raster<Pixel> destination { glm::uvec2(size) };
            detail::copy_mat_to_raster(source, destination);
            return destination;
        } catch (const std::bad_alloc&) {
            return alp::make_error(OutOfMemory {}, context);
        } catch (const cv::Exception& exception) {
            if (exception.code == cv::Error::StsNoMem) {
                return alp::make_error(OutOfMemory {}, context, OpenCvError(exception));
            }
            return alp::make_error(OpenCvError(exception), "copying the matrix into the raster");
        }
    }
}

using ToMatError = alp::Error<Unsupported, OutOfMemory, OpenCvError>;

template <typename Pixel>
alp::Result<cv::Mat, ToMatError> to_mat(const radix::Raster<Pixel>& source)
{
    if constexpr (!detail::Traits<Pixel>::supported) {
        return alp::make_error(Unsupported {}, fmt::format("there is no OpenCV type corresponding to {}", type_name<Pixel>()));
    } else {
        ASSERT(glm::all(glm::lessThanEqual(source.size(), glm::uvec2(std::numeric_limits<int>::max()))));
        const glm::ivec2 size(source.size());

        constexpr auto context = "allocating the matrix";
        try {
            cv::Mat destination(size.y, size.x, detail::cv_type<Pixel>);
            detail::copy_raster_to_mat(source, destination);
            return destination;
        } catch (const std::bad_alloc&) {
            return alp::make_error(OutOfMemory {}, context);
        } catch (const cv::Exception& exception) {
            if (exception.code == cv::Error::StsNoMem) {
                return alp::make_error(OutOfMemory {}, context, OpenCvError(exception));
            }
            return alp::make_error(OpenCvError(exception), "copying the raster into the matrix");
        }
    }
}

} // namespace io::conversion
