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

#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "io/FileError.h"
#include "io/filesystem.h"
#include "raster_store/attribution/Table.h"
#include "raster_store/attribution/table_file.h"
#include "raster_store_test_utils.h"

namespace {

namespace attribution = raster_store::attribution;
using raster_store_test::entity_json;
using raster_store_test::write_table;
using raster_store_test::write_text;

} // namespace

TEST_CASE("attribution tables preserve complete ordinary objects", "[raster_store][attribution][table_file]")
{
    const attribution::Table table = REQUIRE_VALUE(attribution::parse("[" + std::string(entity_json) + "]"));
    REQUIRE(table.entities().size() == 1);
    CHECK(table.entities()[0].spatial_resolution == 0.25);
    CHECK(table.entities()[0].acquisition_date == "summer 2020");

    const attribution::Table ordinary_empty_values = REQUIRE_VALUE(attribution::parse(
        R"([{"spatial_resolution":0,"acquisition_date":"","ingestion_date":"","copyright":"","copyright_link":"","license":""}])"));
    CHECK(ordinary_empty_values.entities()[0].copyright.empty());
}

TEST_CASE("attribution tables reject malformed slots and empty tables", "[raster_store][attribution][table_file]")
{
    for (const std::string_view json : { "{}", "[null]", "[{}]", "[1]", "not json" }) {
        INFO(json);
        REQUIRE(REQUIRE_ERROR(attribution::parse(json)).holds<attribution::CorruptTable>());
    }
    CHECK(REQUIRE_ERROR(attribution::parse("[]")).holds<attribution::EmptyTable>());

    std::string wrong_resolution(entity_json);
    wrong_resolution.replace(wrong_resolution.find("0.25"), 4, "\"0.25\"");
    CHECK(REQUIRE_ERROR(attribution::parse("[" + wrong_resolution + "]")).holds<attribution::CorruptTable>());
    std::string wrong_date(entity_json);
    wrong_date.replace(wrong_date.find("\"summer 2020\""), 13, "null");
    CHECK(REQUIRE_ERROR(attribution::parse("[" + wrong_date + "]")).holds<attribution::CorruptTable>());
}

TEST_CASE("attribution lookup stops at the nearest of exactly three locations", "[raster_store][attribution][table_file]")
{
    const TempDirectory directory("attribution-lookup");
    const std::filesystem::path grandparent = directory.path() / "collection";
    const std::filesystem::path parent = grandparent / "layer";
    const std::filesystem::path storage_path = parent / "snapshot";
    write_table(directory.path());
    CHECK(REQUIRE_ERROR(attribution::find_table(storage_path)).holds<attribution::MissingTable>());
    write_table(grandparent);
    CHECK(REQUIRE_VALUE(attribution::find_table(storage_path)) == grandparent / attribution::file_name);
    write_table(parent);
    CHECK(REQUIRE_VALUE(attribution::find_table(storage_path)) == parent / attribution::file_name);
    write_table(storage_path);
    CHECK(REQUIRE_VALUE(attribution::find_table(storage_path)) == storage_path / attribution::file_name);
    write_text(storage_path / attribution::file_name, "[null]");
    CHECK(REQUIRE_ERROR(attribution::read_table(storage_path)).holds<attribution::CorruptTable>());
    REQUIRE(REQUIRE_VALUE(io::filesystem::remove(storage_path / attribution::file_name)));
    REQUIRE_VALUE(io::filesystem::create_symlink(storage_path / "missing", storage_path / attribution::file_name));
    CHECK(REQUIRE_VALUE(attribution::find_table(storage_path)) == storage_path / attribution::file_name);
    REQUIRE_ERROR(attribution::read_table(storage_path));
}

TEST_CASE("attribution copy reports missing paths", "[raster_store][attribution][table_file]")
{
    const TempDirectory directory("attribution-copy");
    const std::filesystem::path source = directory.path() / "source";
    const std::filesystem::path destination = directory.path() / "destination";
    write_table(source);
    SECTION("dangling source symlink")
    {
        REQUIRE(REQUIRE_VALUE(io::filesystem::remove(source / attribution::file_name)));
        REQUIRE_VALUE(io::filesystem::create_symlink(source / "missing", source / attribution::file_name));
        REQUIRE(REQUIRE_VALUE(io::filesystem::create_directory(destination)));
    }
    SECTION("missing destination directory") { }

    const auto error = REQUIRE_ERROR(attribution::copy_table(source, destination));
    CHECK(error.holds_and<io::FileError>([](const io::FileError& e) { return e.code() == std::errc::no_such_file_or_directory; }));
}
