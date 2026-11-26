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

#include <utility>

#include <libassert/assert.hpp>

#include "error/Error.h"
#include "io/compression.h"
#include "io/envelope.h"
#include "io/serde.h"
#include "store/Codec.h"
#include "store/IndexFormat.h"
#include "store/Metadata.h"

namespace store {

// The failure reading a node for a failure reading its envelope, which it keeps as its source.
[[nodiscard]]
inline CodecReadError to_codec_read_error(io::envelope::ReadError error) {
    return std::move(error)
        .map<io::envelope::CorruptData, io::compression::CorruptData, io::compression::Truncated>(CorruptData{})
        .map<io::compression::Unknown>(Unknown{});
}

// The failure writing a node for a failure writing its envelope, which it keeps as its source.
// A failure to serialize the envelope's payload is a bug in the codec and aborts.
[[nodiscard]]
inline CodecWriteError to_codec_write_error(io::envelope::WriteError error) {
    auto unserializable_result = std::move(error).split<io::serde::Unserializable>();
    ASSERT(!unserializable_result.has_value(), "serializing the payload failed, which is a bug in the codec");
    return std::move(unserializable_result).error().map<io::compression::Unknown>(Unknown{});
}

// The failure reading an index file for a failure reading its envelope, which it keeps as its source.
[[nodiscard]]
inline ReadIndexError to_read_index_error(io::envelope::ReadError error) {
    return std::move(error).map<io::envelope::CorruptData, io::compression::CorruptData, io::compression::Truncated, io::compression::Unknown>(CorruptIndex{});
}

// The failure writing an index file for a failure writing its envelope, which it keeps as its source.
[[nodiscard]]
inline WriteIndexError to_write_index_error(io::envelope::WriteError error) {
    auto unserializable_result = std::move(error).split<io::serde::Unserializable>();
    ASSERT(!unserializable_result.has_value(), "the index types have an encoding");
    return std::move(unserializable_result).error().map<io::compression::Unknown>(UnencodableIndex{});
}

// The failure reading a metadata file for a failure reading its envelope, which it keeps as its source.
[[nodiscard]]
inline ReadMetadataError to_read_metadata_error(io::envelope::ReadError error) {
    return std::move(error)
        .map<io::envelope::CorruptData, io::compression::CorruptData, io::compression::Truncated, io::compression::Unknown>(CorruptMetadata{});
}

// The failure writing a metadata file for a failure writing its envelope, which it keeps as its source.
[[nodiscard]]
inline WriteMetadataError to_write_metadata_error(io::envelope::WriteError error) {
    auto unserializable_result = std::move(error).split<io::serde::Unserializable>();
    ASSERT(!unserializable_result.has_value(), "the metadata types have an encoding");
    return std::move(unserializable_result).error().map<io::compression::Unknown>(UnencodableMetadata{});
}

} // namespace store
