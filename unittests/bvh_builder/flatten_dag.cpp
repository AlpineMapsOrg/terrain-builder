#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "dag_helpers.h"
#include "node_header_helpers.h"

#include <cstdint>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "../dag_builder/dag_node.h"
#include "../dag_builder/storage.h"
#include "range_utils.h"

#include "bvh/Batch.h"
#include "bvh/Node.h"
#include "bvh/NodeHeader.h"
#include "bvh/NodeInput.h"
#include "bvh/NodeView.h"
#include "bvh/Storage.h"
#include "bvh/flatten_dag.h"
#include "dag/SourceId.h"

namespace {

// Flattens a dag holding exactly `dag_nodes` and returns every node written, each of which must be a leaf.
std::vector<bvh::LeafInput> flatten(std::vector<std::pair<octree::Id, dag::NodeMetadata>> dag_nodes) {
    const TempDirectory dag_directory("flatten_dag_input");
    write_dag(dag_directory.path() / "dag", std::move(dag_nodes));
    const auto dag = REQUIRE_VALUE(dag::storage::open_metadata(dag_directory.path() / "dag"));

    const TempDirectory bvh_directory("flatten_dag_output");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(bvh_directory.path()));
    REQUIRE_VALUE(bvh::flatten_dag(dag, nodes));
    REQUIRE_VALUE(nodes.save_index());

    std::vector<bvh::LeafInput> leaves;
    for (const uint32_t batch_index : range(nodes.size())) {
        const bvh::Batch batch = REQUIRE_VALUE(nodes.read(batch_index));
        for (const bvh::Node &node : batch.nodes()) {
            const bvh::NodeView view = batch.resolve(node);
            REQUIRE(view.is_leaf());
            leaves.push_back(bvh::LeafInput{.header = view.header(), .source = view.source()});
        }
    }
    return leaves;
}

}

TEST_CASE("flatten_dag writes a leaf per group, including root groups", "[bvh][flatten_dag]") {
    const octree::Id coarse = octree::Id::root();
    const octree::Id fine = coarse.child(0).value();
    const bvh::Bounds box{glm::dvec3{0.0}, glm::dvec3{1.0}};

    const std::vector<bvh::LeafInput> leaves = flatten({
        {coarse, {.clusters = {{.group_index = 0, .bounds = box}},
                  .groups = {{.children = {{fine, 0}, {fine, 1}}, .error = 3.0, .bounds = box}}}},
        {fine, {.clusters = {{.group_index = std::nullopt, .bounds = box}, {.group_index = std::nullopt, .bounds = box}}, .groups = {}}},
    });

    CHECK_THAT(leaves, Catch::Matchers::UnorderedEquals(std::vector<bvh::LeafInput>{
        {make_node_header({0.0, 3.0}, box), dag::GroupId{coarse, 0}},
        {make_node_header({3.0, bvh::ROOT_PARENT_ERROR}, box), dag::ClusterId{coarse, 0}},
    }));
}

TEST_CASE("flatten_dag measures a leaf from its group's box and bounds its clusters tightly", "[bvh][flatten_dag]") {
    const octree::Id coarse = octree::Id::root();
    const octree::Id fine = coarse.child(0).value();
    const octree::Id fine_child = fine.child(0).value();
    // The group's box is larger than its clusters', so the LOD and tight bounds differ.
    const bvh::Bounds group_box{glm::dvec3{0.0}, glm::dvec3{8.0}};
    const bvh::Bounds coarse_box{glm::dvec3{2.0}, glm::dvec3{6.0}};
    const bvh::Bounds first_box{glm::dvec3{1.0}, glm::dvec3{3.0}};
    const bvh::Bounds second_box{glm::dvec3{5.0}, glm::dvec3{7.0}};
    const bvh::Bounds both_boxes{glm::dvec3{1.0}, glm::dvec3{7.0}};

    // The first child has the larger error, so taking the first child's error is wrong.
    const std::vector<bvh::LeafInput> leaves = flatten({
        {coarse, {.clusters = {{.group_index = 0, .bounds = coarse_box}},
                  .groups = {{.children = {{fine, 0}, {fine, 1}}, .error = 3.0, .bounds = group_box}}}},
        {fine, {.clusters = {{.group_index = 0, .bounds = first_box}, {.group_index = 1, .bounds = second_box}},
                .groups = {{.children = {{fine_child, 0}}, .error = 2.0, .bounds = first_box}, {.children = {{fine_child, 1}}, .error = 1.0, .bounds = second_box}}}},
        {fine_child, {.clusters = {{.group_index = std::nullopt, .bounds = first_box}, {.group_index = std::nullopt, .bounds = second_box}}, .groups = {}}},
    });

    CHECK_THAT(to_vector(leaves | std::views::transform(&bvh::LeafInput::header)), Catch::Matchers::UnorderedEquals(std::vector{
        make_node_header({1.0, 3.0}, group_box, both_boxes),
        make_node_header({3.0, bvh::ROOT_PARENT_ERROR}, group_box, coarse_box),
        make_node_header({0.0, 2.0}, first_box, first_box),
        make_node_header({0.0, 1.0}, second_box, second_box),
    }));
}
