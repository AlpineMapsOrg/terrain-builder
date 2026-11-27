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

#include <algorithm>
#include <array>
#include <filesystem>
#include <ranges>
#include <span>
#include <string_view>

#include <fmt/format.h>
#include <fmt/std.h>
#include <libassert/assert.hpp>
#include <magic_enum/magic_enum.hpp>

#include "array_utils.h"
#include "common_error_codes.h"
#include "error/Result.h"
#include "error/try.h"
#include "fmt_impls.h"
#include "io/UnknownExtension.h"
#include "range_utils.h"
#include "string_utils.h"

namespace io::image {

// The format of an encoded image.
enum class Format {
    Jpeg,
    Png,
    JpegXl,
};

inline std::string_view format_as(const Format format) {
    return magic_enum::enum_name(format);
}

// The extensions, with their dot, that name `format`.
constexpr std::span<const std::string_view> to_extensions(const Format format) {
    switch (format) {
    case Format::Jpeg: {
        static constexpr std::array<std::string_view, 2> result = {".jpeg", ".jpg"};
        return result;
    }
    case Format::Png: {
        static constexpr std::array<std::string_view, 1> result = {".png"};
        return result;
    }
    case Format::JpegXl: {
        static constexpr std::array<std::string_view, 1> result = {".jxl"};
        return result;
    }
    }
    UNREACHABLE("invalid image format", format);
}

// The extension, with its dot, that names `format`.
constexpr std::string_view to_extension(const Format format) {
    return to_extensions(format).front();
}

// The extensions format_from_extension recognises.
inline constexpr auto extensions = [] {
    static constexpr auto extensions_by_format = transform_array(magic_enum::enum_values<Format>(), to_extensions);
    constexpr size_t extension_count = sum(extensions_by_format, std::ranges::size);
    std::array<std::string_view, extension_count> result;
    auto output = result.begin();
    for (const std::span<const std::string_view> format_extensions : extensions_by_format) {
        output = std::ranges::copy(format_extensions, output).out;
    }
    return result;
}();

// The format that `extension`, with its dot, names, ignoring ASCII case.
inline alp::Result<Format, Unsupported> format_from_extension(const std::string_view extension) {
    for (const Format format : magic_enum::enum_values<Format>()) {
        // TODO: switch back to std::bind_front once CI no longer builds with clang against libstdc++ 14.2
        if (std::ranges::any_of(to_extensions(format), [&](const std::string_view e) { return equals_ignore_ascii_case(extension, e); })) {
            return format;
        }
    }
    return make_unsupported_extension_error(extension, extensions);
}

// The format that the extension of `path` names, ignoring ASCII case.
inline alp::Result<Format, Unsupported> format_from_path(const std::filesystem::path &path) {
    ALP_TRY_CONTEXT(const Format format, format_from_extension(path.extension().string()), fmt::format("choosing the image format for {}", path));
    return format;
}

// The media type of `format`.
inline std::string_view to_mime_type(const Format format) {
    switch (format) {
    case Format::Jpeg:
        return "image/jpeg";
    case Format::Png:
        return "image/png";
    case Format::JpegXl:
        return "image/jxl";
    }
    UNREACHABLE("invalid image format", format);
}

} // namespace io::image
