#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "node_header_helpers.h"

#include <cstdint>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "../dag_builder/dag_node.h"
#include "../dag_builder/encoded.h"
#include "../dag_builder/storage.h"
#include "octree/storage/open.h"
#include "range_utils.h"

#include "bvh/Batch.h"
#include "bvh/Node.h"
#include "bvh/NodeHeader.h"
#include "bvh/NodeInput.h"
#include "bvh/NodeView.h"
#include "bvh/Storage.h"
#include "bvh/flatten_dag.h"

namespace {

// A leaf as read back.
using Leaf = bvh::LeafInput<std::vector<dag::ClusterInfo>>;

// Flattens a dag holding exactly `dag_nodes` and returns every node written, each of which must be a leaf.
std::vector<Leaf> flatten(std::vector<std::pair<octree::Id, dag::NodeMetadata>> dag_nodes) {
    const TempDirectory dag_directory("flatten_dag_input");
    {
        auto dag = REQUIRE_VALUE(octree::open_folder_indexed<dag::ClusterBatch>(dag_directory.path()));
        for (auto &[id, metadata] : dag_nodes) {
            REQUIRE(dag.save(id, dag::ClusterBatch{std::move(metadata), Clustering{}}).has_value());
        }
        REQUIRE(dag.save_index().has_value());
    }
    const auto dag = REQUIRE_VALUE(octree::open_folder_indexed<dag::NodeMetadata>(dag_directory.path()));

    const TempDirectory bvh_directory("flatten_dag_output");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(bvh_directory.path()));
    REQUIRE(bvh::flatten_dag(dag, nodes).has_value());
    REQUIRE(nodes.save_index().has_value());

    std::vector<Leaf> leaves;
    for (const uint32_t batch_index : range(nodes.size())) {
        const bvh::Batch batch = REQUIRE_VALUE(nodes.read(batch_index));
        for (const bvh::Node &node : batch.nodes()) {
            const bvh::NodeView view = batch.resolve(node);
            REQUIRE(view.is_leaf());
            leaves.push_back(Leaf{.header = view.header(), .clusters = to_vector(view.clusters())});
        }
    }
    return leaves;
}

}

TEST_CASE("flatten_dag writes a leaf per group, including root groups", "[bvh][flatten_dag]") {
    const octree::Id coarse = octree::Id::root();
    const octree::Id fine = coarse.child(0).value();
    const bvh::Bounds box{glm::dvec3{0.0}, glm::dvec3{1.0}};

    const std::vector<Leaf> leaves = flatten({
        {coarse, {.clusters = {{.group_index = 0, .bounds = box}},
                  .groups = {{.children = {{fine, 0}, {fine, 1}}, .error = 3.0, .bounds = box}}}},
        {fine, {.clusters = {{.group_index = std::nullopt, .bounds = box}, {.group_index = std::nullopt, .bounds = box}}, .groups = {}}},
    });

    CHECK_THAT(leaves, Catch::Matchers::UnorderedEquals(std::vector<Leaf>{
        {make_node_header({0.0, 3.0}, box), {{.id = {fine, 0}, .lod_bounds = box, .tight_bounds = box}, {.id = {fine, 1}, .lod_bounds = box, .tight_bounds = box}}},
        {make_node_header({3.0, bvh::ROOT_PARENT_ERROR}, box), {{.id = {coarse, 0}, .error = 3.0, .lod_bounds = box, .tight_bounds = box}}},
    }));
}

TEST_CASE("flatten_dag measures a leaf from its group's box and bounds its clusters tightly", "[bvh][flatten_dag]") {
    const octree::Id coarse = octree::Id::root();
    const octree::Id fine = coarse.child(0).value();
    // The group's box is larger than its clusters', so the LOD and tight bounds differ.
    const bvh::Bounds group_box{glm::dvec3{0.0}, glm::dvec3{8.0}};
    const bvh::Bounds coarse_box{glm::dvec3{2.0}, glm::dvec3{6.0}};
    const bvh::Bounds first_box{glm::dvec3{1.0}, glm::dvec3{3.0}};
    const bvh::Bounds second_box{glm::dvec3{5.0}, glm::dvec3{7.0}};
    const bvh::Bounds both_boxes{glm::dvec3{1.0}, glm::dvec3{7.0}};

    // The first child has the larger error, so taking the first child's error is wrong.
    const std::vector<Leaf> leaves = flatten({
        {coarse, {.clusters = {{.group_index = 0, .bounds = coarse_box}},
                  .groups = {{.children = {{fine, 0}, {fine, 1}}, .error = 3.0, .bounds = group_box}}}},
        {fine, {.clusters = {{.group_index = 0, .bounds = first_box}, {.group_index = 1, .bounds = second_box}},
                .groups = {{.error = 2.0, .bounds = first_box}, {.error = 1.0, .bounds = second_box}}}},
    });

    CHECK_THAT(to_vector(leaves | std::views::transform(&Leaf::header)), Catch::Matchers::UnorderedEquals(std::vector{
        make_node_header({1.0, 3.0}, group_box, both_boxes),
        make_node_header({3.0, bvh::ROOT_PARENT_ERROR}, group_box, coarse_box),
    }));
}
