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

#pragma once

#include <optional>
#include <string>

#include <fmt/format.h>
#include <opencv2/core.hpp>

namespace io {

// A failure OpenCV reported, by throwing cv::Exception or by returning failure without a reason.
struct OpenCvError {
    // What OpenCV's exception says.
    struct Info {
        // The cv::Error code.
        int code;
        // OpenCV's description of the failure.
        std::string description;

        bool operator==(const Info &) const = default;
    };

    OpenCvError() = default;
    explicit OpenCvError(const cv::Exception &exception) : exception(Info{.code = exception.code, .description = exception.err}) {}

    // The exception OpenCV threw, if it threw one.
    std::optional<Info> exception;

    bool operator==(const OpenCvError &) const = default;
};

inline std::string format_as(const OpenCvError &error) {
    if (error.exception) {
        return fmt::format("{} (OpenCV error code {})", error.exception->description, error.exception->code);
    } else {
        return "OpenCV reported a failure without a reason";
    }
}

} // namespace io
