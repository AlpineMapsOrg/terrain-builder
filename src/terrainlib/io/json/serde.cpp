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

#include "io/json/serde.h"

#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <variant>

#include <cpl_json.h>
#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "enumerate.h"
#include "error/prelude.h"
#include "variant_utils.h"

namespace io::json {

namespace {

// One step of the path from the root to the value being written.
struct PointerSegment {
    const PointerSegment *parent;
    std::variant<size_t, std::string_view> token;
};

// `token` escaped as a JSON pointer reference token, as RFC 6901 requires.
std::string escape_pointer_token(const std::string_view token) {
    std::string result;
    for (const char c : token) {
        if (c == '~') {
            result += "~0";
        } else if (c == '/') {
            result += "~1";
        } else {
            result += c;
        }
    }
    return result;
}

// The JSON pointer from the root to `segment`, or the empty pointer for the root itself.
std::string format_pointer(const PointerSegment *segment) {
    if (segment == nullptr) {
        return "";
    }
    const std::string token = match(
        segment->token,
        [](const size_t index) { return fmt::to_string(index); },
        [](const std::string_view key) { return escape_pointer_token(key); });
    std::string result = format_pointer(segment->parent);
    result += '/';
    result += token;
    return result;
}

// Fails if `text`, the string or key at `segment`, contains a NUL character, which the backend cannot represent.
alp::Result<void, Unsupported> check_no_nul(const std::string_view text, const PointerSegment *segment) {
    if (text.find('\0') != std::string_view::npos) {
        const std::string pointer = format_pointer(segment);
        return alp::make_error(Unsupported{}, fmt::format("writing a string containing a NUL character at JSON pointer '{}'", pointer));
    }
    return {};
}

} // namespace

// Converts values to the backend; a friend of Value, so that it can match on what a value holds.
struct Serde {
    using BackendResult = alp::Result<CPLJSONObject, Unsupported>;

    // `value`, located at `segment`, as a backend object.
    static BackendResult to_backend(const Value &value, const PointerSegment *segment) {
        return match<BackendResult>(
            value._value,
            [](std::nullptr_t) { return CPLJSONObject(nullptr); },
            [](const bool boolean) { return CPLJSONObject(boolean); },
            [](const int64_t integer) { return CPLJSONObject(integer); },
            [](const uint64_t integer) { return CPLJSONObject(integer); },
            [&](const double number) -> BackendResult {
                if (!std::isfinite(number)) {
                    const std::string pointer = format_pointer(segment);
                    return alp::make_error(Unsupported{}, fmt::format("writing the non-finite number {} at JSON pointer '{}'", number, pointer));
                }
                return CPLJSONObject(number);
            },
            [&](const std::string &text) -> BackendResult {
                ALP_TRY(check_no_nul(text, segment));
                return CPLJSONObject(text);
            },
            [&](const alp::indirect<Array> &array) -> BackendResult {
                CPLJSONArray result;
                for (const auto &[index, item] : enumerate(*array)) {
                    const PointerSegment item_segment{segment, index};
                    ALP_TRY(const CPLJSONObject item_backend, to_backend(item, &item_segment));
                    // Add skips a null object instead of adding a JSON null.
                    if (item.kind() == Kind::Null) {
                        result.AddNull();
                    } else {
                        result.Add(item_backend);
                    }
                }
                return result;
            },
            [&](const alp::indirect<Object> &object) -> BackendResult {
                CPLJSONObject result;
                for (const auto &[key, item] : *object) {
                    const PointerSegment item_segment{segment, key};
                    ALP_TRY(check_no_nul(key, &item_segment));
                    ALP_TRY(const CPLJSONObject item_backend, to_backend(item, &item_segment));
                    // The other Add functions treat '/' in a key as a path separator.
                    result.AddNoSplitName(key, item_backend);
                }
                return result;
            });
    }
};

alp::Result<std::string, Unsupported> to_string(const Value &value, const Format format) {
    ALP_TRY_CONTEXT(const CPLJSONObject backend, Serde::to_backend(value, nullptr), "serializing a JSON value");
    // Format writes a JSON null as an empty string.
    if (value.kind() == Kind::Null) {
        return std::string("null");
    }
    switch (format) {
    case Format::Compact:
        return backend.Format(CPLJSONObject::PrettyFormat::Plain);
    case Format::Pretty:
        return backend.Format(CPLJSONObject::PrettyFormat::Pretty);
    }
    UNREACHABLE();
}

} // namespace io::json
