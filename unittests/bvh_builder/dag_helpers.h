#pragma once

#include "../catch2_helpers.h"
#include "../result_helpers.h"

#include <filesystem>
#include <utility>
#include <vector>

#include "../dag_builder/dag_node.h"
#include "../dag_builder/encoded.h"
#include "../dag_builder/storage.h"
#include "octree/storage/open.h"

// A dag folder holding exactly the given nodes.
inline void write_dag(const std::filesystem::path &path, std::vector<std::pair<octree::Id, dag::NodeMetadata>> nodes) {
    auto dag = REQUIRE_VALUE(octree::open_folder_indexed<dag::ClusterBatch>(path));
    for (auto &[id, metadata] : nodes) {
        REQUIRE_VALUE(dag.save(id, dag::ClusterBatch{std::move(metadata), Clustering{}}));
    }
    REQUIRE_VALUE(dag.save_index());
}
