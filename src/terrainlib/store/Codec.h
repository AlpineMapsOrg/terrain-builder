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

#include <concepts>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <string_view>
#include <type_traits>
#include <vector>

#include "common_error_codes.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/utils.h"
#include "io/FileError.h"
#include "io/alloc.h"
#include "io/bytes.h"
#include "store/Hash.h"

namespace store {

struct CorruptData : alp::MessageCode<"the node's files are not valid for its codec"> {};
struct Unknown : alp::MessageCode<"the codec's library reported a failure of unknown meaning"> {};

using CodecReadError = alp::Error<io::FileError, CorruptData, Unsupported, io::AllocError, Unknown>;
using CodecWriteError = alp::Error<io::FileError, Unsupported, io::AllocError, Unknown>;
using CodecError = alp::Error<CodecReadError, CodecWriteError>;

// How a storage reads and writes the files of one node.
// Implementations must support concurrent calls on the same instance.
// `write` creates the node's files exclusively and fails with a file error if one of them exists; it never truncates or replaces a file, which may be a hard link shared with another storage.
// Reading a node requires its write to have completed.
template <typename Payload>
class Codec {
public:
    virtual ~Codec() = default;

    // The codec id, which names the codec in a storage's metadata.
    // Codecs with equal ids read the same payload from the same files, and the files they write for a payload read back as the same payload.
    virtual std::string_view id() const = 0;

    virtual std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const = 0;

    // Reads the node's payload.
    // If an expected hash is given, it must equal the hash of the node's files.
    [[nodiscard]]
    virtual alp::Result<Payload, CodecReadError> read(const std::filesystem::path&, std::optional<PayloadHashView> /* expected_hash */ = std::nullopt) const {
        return alp::make_error(Unsupported{}, "the codec cannot read");
    }

    // Writes the node's payload and returns its hash, or an empty hash if the codec does not support hashing.
    [[nodiscard]]
    virtual alp::Result<PayloadHash, CodecWriteError> write(const std::filesystem::path&, const Payload&) const {
        return alp::make_error(Unsupported{}, "the codec cannot write");
    }

    // Whether `write` returns the payload's hash, of at least min_hash_size bytes, and `read` checks an expected one.
    virtual bool supports_hashing() const { return false; }
};

// A function that gives the codec a storage's codec id names, failing with Unsupported for an id it cannot serve.
template <typename F, typename Payload>
concept CodecResolver = std::invocable<F&, std::string_view>
    && std::same_as<std::invoke_result_t<F&, std::string_view>, alp::Result<std::unique_ptr<Codec<Payload>>, Unsupported>>;

} // namespace store
