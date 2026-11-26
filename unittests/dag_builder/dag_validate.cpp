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

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "dag_validate.h"

TEST_CASE("validate accepts a consistent batch", "[dag_builder][dag_validate]") {
    dag::ClusterBatch batch;
    batch.metadata = { .group_assignment = { 0 }, .triangle_counts = { 1 }, .groups = { { .children = {}, .error = 0.5, .bounds = {} } } };
    batch.clustering.positions = { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } };
    batch.clustering.clusters = { { .id = 0, .vertex_indices = { 0, 1, 2 }, .local_triangles = { { 0, 1, 2 } }, .uvs = {}, .texture_id = std::nullopt, .absolute_error = 0.5 } };

    REQUIRE_VALUE(dag::validate(batch));
    REQUIRE_VALUE(dag::validate(batch.metadata));
}

TEST_CASE("validate rejects inconsistent metadata", "[dag_builder][dag_validate]") {
    // One cluster in one group.
    dag::NodeMetadata metadata = { .group_assignment = { 0 }, .triangle_counts = { 1 }, .groups = { { .children = {}, .error = 0.5, .bounds = {} } } };
    SECTION("group assignment names no group") {
        metadata.group_assignment = { 1 };
    }
    SECTION("no triangle count per group assignment") {
        metadata.triangle_counts.clear();
    }
    SECTION("group error not finite") {
        metadata.groups.front().error = std::numeric_limits<double>::quiet_NaN();
    }

    const auto error = REQUIRE_ERROR(dag::validate(metadata));

    CHECK(error.holds<dag::InconsistentMetadata>());
}

TEST_CASE("validate rejects inconsistent batches", "[dag_builder][dag_validate]") {
    // Three positions, one cluster of three vertices in one group.
    dag::ClusterBatch batch;
    batch.metadata = { .group_assignment = { 0 }, .triangle_counts = { 1 }, .groups = { { .children = {}, .error = 0.5, .bounds = {} } } };
    batch.clustering.positions = { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } };
    batch.clustering.clusters = { { .id = 0, .vertex_indices = { 0, 1, 2 }, .local_triangles = { { 0, 1, 2 } }, .uvs = {}, .texture_id = std::nullopt, .absolute_error = 0.5 } };
    SECTION("metadata inconsistent") {
        batch.metadata.group_assignment = { 1 };
    }
    SECTION("clustering inconsistent") {
        batch.clustering.clusters.front().vertex_indices = { 0, 1, 3 };
    }
    SECTION("no group assignment per cluster") {
        batch.metadata.group_assignment = { 0, 0 };
        batch.metadata.triangle_counts = { 1, 1 };
    }

    const auto error = REQUIRE_ERROR(dag::validate(batch));

    CHECK(error.holds<dag::InconsistentBatch>());
}
