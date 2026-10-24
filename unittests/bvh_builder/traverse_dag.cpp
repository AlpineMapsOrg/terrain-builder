#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "../dag_builder/dag_node.h"
#include "../dag_builder/encoded.h"
#include "../dag_builder/storage.h"
#include "dag/traverse_dag.h"
#include "octree/storage/open.h"

namespace {

constexpr size_t AMPLE_RESIDENT_BYTES = 1024 * 1024;

// A box that no other `index` gives.
radix::geometry::Aabb3d make_distinct_box(const double index) {
    return {glm::dvec3{2.0 * index}, glm::dvec3{2.0 * index + 1.0}};
}

// A dag folder holding exactly the given nodes.
void write_dag(const std::filesystem::path &path, std::vector<std::pair<octree::Id, dag::NodeMetadata>> nodes) {
    auto dag = REQUIRE_VALUE(octree::open_folder_indexed<dag::ClusterBatch>(path));
    for (auto &[id, metadata] : nodes) {
        REQUIRE(dag.save(id, dag::ClusterBatch{std::move(metadata), Clustering{}}).has_value());
    }
    REQUIRE(dag.save_index().has_value());
}

// Every group the walk over the dag at `path` visits, with its clusters ordered by id.
std::vector<dag::GroupInfo> collect_groups(const std::filesystem::path &path, const size_t max_resident_bytes) {
    const auto dag = REQUIRE_VALUE(octree::open_folder_indexed<dag::NodeMetadata>(path));
    std::vector<dag::GroupInfo> visited;
    const auto walked = dag::traverse_groups(dag, max_resident_bytes, [&](const dag::GroupView &group) {
        dag::GroupInfo owned = group.to_owned();
        // The walk promises no order within a group.
        std::ranges::sort(owned.child_clusters, {}, &dag::ClusterInfo::id);
        std::ranges::sort(owned.parent_clusters, {}, &dag::ClusterInfo::id);
        visited.push_back(std::move(owned));
    });
    REQUIRE(walked.value_or(false));
    return visited;
}

}

TEST_CASE("traverse_groups visits every group once, including root groups", "[dag][traverse_dag]") {
    // Small enough for reading a child's node to evict the node of the group listing it.
    const size_t max_resident_bytes = GENERATE(AMPLE_RESIDENT_BYTES, size_t{1});

    const octree::Id root = octree::Id::root();
    const octree::Id first_child = root.child(0).value();
    const octree::Id second_child = root.child(1).value();
    const octree::Id grandchild = first_child.child(0).value();

    // The root's two groups alternate in its clusters, and its first group lists clusters of two nodes.
    const dag::NodeMetadata root_node{
        .clusters = {
            {.group_index = 1, .bounds = make_distinct_box(0)},
            {.group_index = 0, .bounds = make_distinct_box(1)},
            {.group_index = 1, .bounds = make_distinct_box(2)},
            {.group_index = std::nullopt, .bounds = make_distinct_box(3)},
        },
        .groups = {
            {.children = {{first_child, 0}, {second_child, 0}}, .error = 3.0, .bounds = make_distinct_box(4)},
            {.children = {{second_child, 1}}, .error = 2.0, .bounds = make_distinct_box(5)},
        },
    };
    const dag::NodeMetadata first_child_node{
        .clusters = {{.group_index = 0, .bounds = make_distinct_box(6)}},
        .groups = {{.children = {{grandchild, 0}}, .error = 1.0, .bounds = make_distinct_box(7)}},
    };
    const dag::NodeMetadata second_child_node{
        .clusters = {{.group_index = std::nullopt, .bounds = make_distinct_box(8)}, {.group_index = std::nullopt, .bounds = make_distinct_box(9)}},
        .groups = {},
    };
    const dag::NodeMetadata grandchild_node{
        .clusters = {{.group_index = std::nullopt, .bounds = make_distinct_box(10)}},
        .groups = {},
    };
    const TempDirectory directory("traverse_dag");
    write_dag(directory.path(), {{root, root_node}, {first_child, first_child_node}, {second_child, second_child_node}, {grandchild, grandchild_node}});

    const dag::ClusterInfo root_0{.id = {root, 0}, .error = root_node.groups[1].error, .lod_bounds = root_node.groups[1].bounds, .tight_bounds = root_node.clusters[0].bounds};
    const dag::ClusterInfo root_1{.id = {root, 1}, .error = root_node.groups[0].error, .lod_bounds = root_node.groups[0].bounds, .tight_bounds = root_node.clusters[1].bounds};
    const dag::ClusterInfo root_2{.id = {root, 2}, .error = root_node.groups[1].error, .lod_bounds = root_node.groups[1].bounds, .tight_bounds = root_node.clusters[2].bounds};
    const dag::ClusterInfo root_3{.id = {root, 3}, .error = 0.0, .lod_bounds = root_node.clusters[3].bounds, .tight_bounds = root_node.clusters[3].bounds};
    const dag::ClusterInfo first_child_0{.id = {first_child, 0}, .error = first_child_node.groups[0].error, .lod_bounds = first_child_node.groups[0].bounds, .tight_bounds = first_child_node.clusters[0].bounds};
    const dag::ClusterInfo second_child_0{.id = {second_child, 0}, .error = 0.0, .lod_bounds = second_child_node.clusters[0].bounds, .tight_bounds = second_child_node.clusters[0].bounds};
    const dag::ClusterInfo second_child_1{.id = {second_child, 1}, .error = 0.0, .lod_bounds = second_child_node.clusters[1].bounds, .tight_bounds = second_child_node.clusters[1].bounds};
    const dag::ClusterInfo grandchild_0{.id = {grandchild, 0}, .error = 0.0, .lod_bounds = grandchild_node.clusters[0].bounds, .tight_bounds = grandchild_node.clusters[0].bounds};

    const std::vector<dag::GroupInfo> groups = collect_groups(directory.path(), max_resident_bytes);
    CHECK_THAT(groups, Catch::Matchers::UnorderedEquals(std::vector<dag::GroupInfo>{
        {.id = dag::GroupId{root, 0}, .error = root_node.groups[0].error, .bounds = root_node.groups[0].bounds, .child_clusters = {first_child_0, second_child_0}, .parent_clusters = {root_1}},
        {.id = dag::GroupId{root, 1}, .error = root_node.groups[1].error, .bounds = root_node.groups[1].bounds, .child_clusters = {second_child_1}, .parent_clusters = {root_0, root_2}},
        {.id = dag::GroupId{first_child, 0}, .error = first_child_node.groups[0].error, .bounds = first_child_node.groups[0].bounds, .child_clusters = {grandchild_0}, .parent_clusters = {first_child_0}},
        {.id = std::nullopt, .error = std::numeric_limits<double>::infinity(), .bounds = root_node.groups[0].bounds, .child_clusters = {root_1}, .parent_clusters = {}},
        {.id = std::nullopt, .error = std::numeric_limits<double>::infinity(), .bounds = root_node.groups[1].bounds, .child_clusters = {root_0, root_2}, .parent_clusters = {}},
        {.id = std::nullopt, .error = std::numeric_limits<double>::infinity(), .bounds = root_node.clusters[3].bounds, .child_clusters = {root_3}, .parent_clusters = {}},
    }));
}

TEST_CASE("traverse_groups stops when the visitor returns false", "[dag][traverse_dag]") {
    const octree::Id root = octree::Id::root();
    const octree::Id child = root.children().value()[0];
    const radix::geometry::Aabb3d bounds{glm::dvec3{0.0}, glm::dvec3{1.0}};

    const TempDirectory directory("traverse_dag");
    write_dag(directory.path(), {
        {root, dag::NodeMetadata{.clusters = {{.group_index = 0, .bounds = bounds}}, .groups = {{.children = {{child, 0}}, .error = 1.0, .bounds = bounds}}}},
        {child, dag::NodeMetadata{.clusters = {{.group_index = std::nullopt, .bounds = bounds}}, .groups = {}}},
    });
    const auto dag = REQUIRE_VALUE(octree::open_folder_indexed<dag::NodeMetadata>(directory.path()));

    uint32_t visit_count = 0;
    const bool walked = REQUIRE_VALUE(dag::traverse_groups(dag, AMPLE_RESIDENT_BYTES, [&](const dag::GroupView &) {
        visit_count++;
        return false;
    }));

    CHECK_FALSE(walked);
    CHECK(visit_count == 1);
}
