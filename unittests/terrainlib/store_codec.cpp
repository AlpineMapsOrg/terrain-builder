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

#include <atomic>
#include <filesystem>
#include <string_view>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "common_error_codes.h"
#include "path_utils.h"
#include "store/Codec.h"
#include "store/Hash.h"

namespace {

class MultiFileCodec final : public store::Codec<int> {
public:
    std::string_view id() const override { return "test.multi"; }

    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override
    {
        return { ::paths::with_added_extension(node_path, ".data"), ::paths::with_added_extension(node_path, ".metadata") };
    }
};

class WriteOnlyCodec final : public store::Codec<int> {
public:
    std::string_view id() const override { return "test.write_only"; }

    std::vector<std::filesystem::path> paths(const std::filesystem::path& node_path) const override { return { ::paths::with_added_extension(node_path, ".out") }; }

    alp::Result<store::PayloadHash, store::CodecWriteError> write(const std::filesystem::path&, const int&) const override
    {
        this->writes++;
        return store::PayloadHash {};
    }

    mutable std::atomic_uint32_t writes = 0;
};

} // namespace

TEST_CASE("runtime codec defaults report unsupported operations", "[store][Codec]")
{
    MultiFileCodec codec;
    const std::filesystem::path path("12/34/56/78");
    CHECK(codec.paths(path)
        == std::vector<std::filesystem::path> {
            "12/34/56/78.data",
            "12/34/56/78.metadata",
        });

    CHECK(REQUIRE_ERROR(codec.read(path)).holds<Unsupported>());
    CHECK(REQUIRE_ERROR(codec.write(path, 42)).holds<Unsupported>());
}

TEST_CASE("runtime write-only codec remains explicitly unreadable", "[store][Codec]")
{
    WriteOnlyCodec codec;
    REQUIRE_VALUE(codec.write("node", 42));
    CHECK(codec.writes == 1);
    CHECK(REQUIRE_ERROR(codec.read("node")).holds<Unsupported>());
}
