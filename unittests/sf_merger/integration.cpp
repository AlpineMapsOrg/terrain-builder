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

#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <filesystem>
#include <string>

#include "../result_helpers.h"
#include "../temp_directory.h"
#include "io/filesystem.h"
#include "mesh/io.h"

namespace {

std::string shell_quote(const std::filesystem::path &path) {
    std::string quoted = "'";
    for (const char character : path.string()) {
        if (character == '\'') {
            quoted += "'\\''";
        } else {
            quoted += character;
        }
    }
    return quoted + "'";
}

int build_sf(
    const std::filesystem::path &dataset,
    const std::filesystem::path &textures,
    const std::filesystem::path &output) {
    const std::string command =
        shell_quote(ALP_SF_BUILDER_PATH) +
        " --dataset " + shell_quote(dataset) +
        " --textures " + shell_quote(textures) +
        " --min-texture-level 12 --max-texture-level 17"
        " --mesh-srs EPSG:4978 --verbosity warn"
        " batch --target-level 15 --output " + shell_quote(output) +
        " --format .sfmesh --threads 1";
    return std::system(command.c_str());
}

} // namespace

TEST_CASE("SF builders preserve geometry at regular and point-touching mask borders", "[integration][sf-builder][sf-merger]") {
    const std::filesystem::path fixture =
        std::filesystem::path(ALP_TEST_DATA_DIR) / "sf_builder_merge_border";
    const TempDirectory temporary_directory("sf_merger_integration");
    const std::filesystem::path base_output = temporary_directory.path() / "basemap-gs";
    const std::filesystem::path new_output = temporary_directory.path() / "gataki-gt";
    const std::filesystem::path merged_output = temporary_directory.path() / "merged";

    REQUIRE(build_sf(
                fixture / "elevation/gs.tif",
                fixture / "orthophoto/basemap",
                base_output) == 0);
    REQUIRE(build_sf(
                fixture / "elevation/gt.tif",
                fixture / "orthophoto/gataki",
                new_output) == 0);

    const std::string merge_command =
        shell_quote(ALP_SF_MERGER_PATH) +
        " merge --base " + shell_quote(base_output) +
        " --new " + shell_quote(new_output) +
        " --mask " + shell_quote(fixture / "mask/tirol.shp") +
        " --output " + shell_quote(merged_output) +
        " --verbosity warn";
    REQUIRE(std::system(merge_command.c_str()) == 0);

    const std::filesystem::path working_relative = "15/26291/18610/27235.sfmesh";
    const std::filesystem::path regression_relative = "15/26290/18610/27235.sfmesh";
    const std::filesystem::path working_path = merged_output / working_relative;
    const std::filesystem::path regression_path = merged_output / regression_relative;

    const SimpleMesh base_regression_mesh = REQUIRE_VALUE(mesh::io::load_from_path(base_output / regression_relative));
    CHECK_FALSE(base_regression_mesh.is_empty());
    const SimpleMesh new_regression_mesh = REQUIRE_VALUE(mesh::io::load_from_path(new_output / regression_relative));
    CHECK_FALSE(new_regression_mesh.is_empty());

    REQUIRE(REQUIRE_VALUE(io::filesystem::exists(working_path)));
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::equivalent(working_path, base_output / working_relative)));
    CHECK_FALSE(REQUIRE_VALUE(io::filesystem::equivalent(working_path, new_output / working_relative)));

    const SimpleMesh working_mesh = REQUIRE_VALUE(mesh::io::load_from_path(working_path));
    CHECK_FALSE(working_mesh.is_empty());

    REQUIRE(REQUIRE_VALUE(io::filesystem::exists(regression_path)));

    const SimpleMesh regression_mesh = REQUIRE_VALUE(mesh::io::load_from_path(regression_path));
    CHECK_FALSE(regression_mesh.is_empty());
}
