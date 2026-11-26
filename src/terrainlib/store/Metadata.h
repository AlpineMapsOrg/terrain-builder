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

#include <string>
#include <utility>

#include <fmt/format.h>

#include "common_error_codes.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/utils.h"
#include "io/FileError.h"
#include "io/alloc.h"

namespace store {

// The ids a storage records once at creation: how its node files are laid out, what they hold, and how they are encoded.
struct Metadata {
    std::string layout_id;
    std::string payload_class;
    std::string codec_id;

    bool operator==(const Metadata&) const = default;
};

// The storage holds payloads of a class other than the one it is opened as.
struct UnexpectedPayloadClass {
    // The payload class the storage's metadata records.
    std::string found;
    // The payload class of the type it is opened as.
    std::string expected;

    bool operator==(const UnexpectedPayloadClass&) const = default;
};

inline std::string format_as(const UnexpectedPayloadClass& error) {
    return fmt::format("the storage holds payloads of class {}, not {}", error.found, error.expected);
}

struct InvalidMetadata : alp::MessageCode<"the storage metadata is not valid"> {};
// Checks that the layout id, payload class and codec id of `metadata` are not empty.
[[nodiscard]]
inline alp::Result<void, InvalidMetadata> validate(const Metadata& metadata) {
    if (metadata.layout_id.empty()) {
        return alp::make_error(InvalidMetadata{}, "the layout id is empty");
    }
    if (metadata.payload_class.empty()) {
        return alp::make_error(InvalidMetadata{}, "the payload class is empty");
    }
    if (metadata.codec_id.empty()) {
        return alp::make_error(InvalidMetadata{}, "the codec id is empty");
    }
    return {};
}

struct CorruptMetadata : alp::MessageCode<"the storage metadata file is corrupt"> {};
struct UnencodableMetadata : alp::MessageCode<"the storage metadata cannot be encoded"> {};

using ReadMetadataError = alp::Error<io::FileError, io::AllocError, CorruptMetadata, Unsupported>;
using WriteMetadataError = alp::Error<io::FileError, io::AllocError, UnencodableMetadata>;

} // namespace store
