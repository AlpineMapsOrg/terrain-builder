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

#include <limits>
#include <optional>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <opencv2/core.hpp>

#include "../result_helpers.h"
#include "validate.h"

TEST_CASE("validate accepts a consistent clustering", "[dag_builder][validate]") {
    Clustering clustering;
    clustering.positions = { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } };
    clustering.clusters = { { .id = 0, .vertex_indices = { 0, 1, 2 }, .local_triangles = { { 0, 1, 2 } }, .uvs = {}, .texture_id = std::nullopt, .absolute_error = 0.5 } };

    REQUIRE_VALUE(validate(clustering));
}

TEST_CASE("validate rejects inconsistent clusterings", "[dag_builder][validate]") {
    // Three positions, one untextured cluster of three vertices and one triangle, one texture.
    Clustering clustering;
    clustering.positions = { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } };
    clustering.clusters = { { .id = 0, .vertex_indices = { 0, 1, 2 }, .local_triangles = { { 0, 1, 2 } }, .uvs = {}, .texture_id = std::nullopt, .absolute_error = 0.5 } };
    clustering.textures = TextureSet(std::vector<cv::Mat> { cv::Mat(1, 1, CV_8UC3) });
    Cluster& cluster = clustering.clusters.front();
    SECTION("cluster has no triangles") {
        cluster.local_triangles.clear();
    }
    SECTION("vertex index names no position") {
        cluster.vertex_indices = { 0, 1, 3 };
    }
    SECTION("triangle names no vertex of the cluster") {
        cluster.local_triangles = { { 0, 1, 3 } };
    }
    SECTION("degenerate triangle") {
        cluster.local_triangles = { { 0, 1, 1 } };
    }
    SECTION("cluster error not finite") {
        cluster.absolute_error = std::numeric_limits<double>::infinity();
    }
    SECTION("texture id names no texture") {
        cluster.texture_id = 1;
        cluster.uvs = { { 0.0, 0.0 }, { 1.0, 0.0 }, { 0.0, 1.0 } };
    }
    SECTION("UVs without a texture") {
        cluster.uvs = { { 0.0, 0.0 }, { 1.0, 0.0 }, { 0.0, 1.0 } };
    }
    SECTION("neither no UVs nor one per vertex") {
        cluster.texture_id = 0;
        cluster.uvs = { { 0.0, 0.0 }, { 1.0, 0.0 } };
    }

    const auto error = REQUIRE_ERROR(validate(clustering));

    CHECK(error.holds<InconsistentClustering>());
}
