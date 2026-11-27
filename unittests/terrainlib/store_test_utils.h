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
#include <bit>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/FileError.h"
#include "io/bytes.h"
#include "io/envelope.h"
#include "octree/Id.h"
#include "octree/path_layout.h"
#include "octree/storage/Index.h"
#include "octree/storage/index_format.h"
#include "path_utils.h"
#include "store/Checkpoint.h"
#include "store/Codec.h"
#include "store/Draft.h"
#include "store/Hash.h"
#include "store/Storage.h"
#include "store/create.h"
#include "store/open.h"

namespace store_test {

using Draft = store::Draft<octree::storage::Index, int>;
using Storage = store::Storage<octree::storage::Index, int>;
using Checkpoint = store::Checkpoint<octree::storage::Index, int>;

// The bytes of an integer as the test codecs write them.
using IntBytes = std::array<uint8_t, sizeof(int)>;

// The test codecs identify an integer by its bytes, followed by filler up to the hash size.
inline store::PayloadHash int_hash(const int value, const size_t size = sizeof(int))
{
    const IntBytes bytes = std::bit_cast<IntBytes>(value);
    store::PayloadHash hash(bytes.begin(), bytes.end());
    hash.resize(size, 0x5a);
    return hash;
}

inline alp::Result<int, store::CodecReadError> read_int(const std::filesystem::path& path, const std::optional<store::PayloadHashView> expected_hash = std::nullopt)
{
    ALP_TRY(const io::bytes::vector bytes, io::bytes::read(path));
    if (bytes.size() != sizeof(int)) {
        return alp::make_error(store::CorruptData {}, "the test integer has the wrong byte count");
    }
    IntBytes int_bytes;
    std::ranges::copy(bytes, int_bytes.begin());
    const int value = std::bit_cast<int>(int_bytes);
    if (expected_hash.has_value() && !std::ranges::equal(*expected_hash, int_hash(value, expected_hash->size()))) {
        return alp::make_error(store::CorruptData {}, "the test integer does not match its expected hash");
    }
    return value;
}

inline alp::Result<void, io::FileError> write_int(const int value, const std::filesystem::path& path)
{
    const IntBytes bytes = std::bit_cast<IntBytes>(value);
    ALP_TRY(io::bytes::write(bytes, path));
    return {};
}

class IntCodec final : public store::Codec<int> {
public:
    std::string_view id() const override { return "test.int"; }

    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override { return { ::paths::with_added_extension(node_path, ".data") }; }

    alp::Result<int, store::CodecReadError> read(const std::filesystem::path& node_path, std::optional<store::PayloadHashView> expected_hash = std::nullopt) const override
    {
        return read_int(this->paths(node_path).front(), expected_hash);
    }

    alp::Result<store::PayloadHash, store::CodecWriteError> write(const std::filesystem::path& node_path, const int& value) const override
    {
        ALP_TRY(write_int(value, this->paths(node_path).front()));
        return int_hash(value);
    }

    bool supports_hashing() const override { return true; }
};

class MultiFileCodec final : public store::Codec<int> {
public:
    std::string_view id() const override { return "test.multi"; }

    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override
    {
        return { ::paths::with_added_extension(node_path, ".data"), ::paths::with_added_extension(node_path, ".metadata") };
    }

    alp::Result<int, store::CodecReadError> read(const std::filesystem::path& node_path, std::optional<store::PayloadHashView> expected_hash = std::nullopt) const override
    {
        return read_int(this->paths(node_path).front(), expected_hash);
    }

    alp::Result<store::PayloadHash, store::CodecWriteError> write(const std::filesystem::path& node_path, const int& value) const override
    {
        const std::vector<std::filesystem::path> node_paths = this->paths(node_path);
        ALP_TRY(write_int(value, node_paths[0]));
        ALP_TRY(write_int(value + 1, node_paths[1]));
        return int_hash(value);
    }

    bool supports_hashing() const override { return true; }
};

// Creates an empty octree draft at `<final_path>.part`.
inline alp::Result<Draft, store::CreateError> create_draft(const std::filesystem::path& final_path, std::unique_ptr<store::Codec<int>> codec = std::make_unique<IntCodec>())
{
    return store::create<octree::storage::Index, int>(final_path, octree::storage::index_format(), octree::path_layout::flat::layout(), std::move(codec));
}

// Opens the octree storage at `path`, which create_draft made.
inline alp::Result<Storage, store::OpenError> open_storage(const std::filesystem::path& path, const store::Accept accept)
{
    return store::open<octree::storage::Index, int>(path, octree::storage::index_format(), octree::path_layout::flat::layout(), std::make_unique<IntCodec>(), accept);
}

// The inputs a test draft records.
struct Inputs {
    std::string input;
    int setting = 0;

    bool operator==(const Inputs&) const = default;
};

using InputsSchema = io::envelope::PayloadSchema<"test.Inputs", io::envelope::Version<1, Inputs>>;

} // namespace store_test
