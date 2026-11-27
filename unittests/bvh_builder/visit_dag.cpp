#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "dag_helpers.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "../dag_builder/storage.h"
#include "dag/Cluster.h"
#include "dag/Group.h"
#include "dag/NodeMetadataCache.h"
#include "dag/read.h"
#include "dag/visit_dag.h"
#include "octree/storage/open.h"
#include "range_utils.h"

namespace {

constexpr size_t UNBOUNDED_RESIDENT_BYTES = std::numeric_limits<size_t>::max();

// POCO version of dag::Cluster
struct ClusterRecord {
    dag::ClusterId id;
    double error;
    radix::geometry::Aabb3d lod_bounds;
    radix::geometry::Aabb3d tight_bounds;

    bool operator==(const ClusterRecord &) const = default;
};

// POCO version of dag::Group
struct GroupRecord {
    dag::GroupId id;
    double error;
    radix::geometry::Aabb3d lod_bounds;
    std::vector<ClusterRecord> clusters;

    bool operator==(const GroupRecord &) const = default;
};

// A box that no other `index` gives.
radix::geometry::Aabb3d make_distinct_box(const double index) {
    return {glm::dvec3{2.0 * index}, glm::dvec3{2.0 * index + 1.0}};
}

// Copies a cluster into comparable values.
ClusterRecord make_cluster_record(const dag::Cluster &cluster) {
    return ClusterRecord{.id = cluster.id(), .error = cluster.error(), .lod_bounds = cluster.lod_bounds(), .tight_bounds = cluster.tight_bounds()};
}

// Copies a group and its child clusters into comparable values, ordered by id.
GroupRecord make_group_record(const dag::Group &group, dag::NodeMetadataCache &cache) {
    const std::vector<dag::Cluster> clusters = REQUIRE_VALUE(dag::read_clusters(group, cache));
    std::vector<ClusterRecord> cluster_records = transform_vector(clusters, make_cluster_record);
    // The walk promises no order within a group.
    std::ranges::sort(cluster_records, {}, &ClusterRecord::id);
    return GroupRecord{.id = group.id(), .error = group.error(), .lod_bounds = group.lod_bounds(), .clusters = std::move(cluster_records)};
}

// Collects every group of the dag at `path`.
std::vector<GroupRecord> collect_groups(const std::filesystem::path &path) {
    const auto dag = REQUIRE_VALUE(octree::open_folder_indexed<dag::NodeMetadata>(path));
    dag::NodeMetadataCache cache(dag, UNBOUNDED_RESIDENT_BYTES);
    std::vector<GroupRecord> visited;
    const auto walked = dag::visit_groups([&](const dag::Group &group) {
        visited.push_back(make_group_record(group, cache));
    }, cache);
    REQUIRE(REQUIRE_VALUE(walked));
    return visited;
}

}

TEST_CASE("visit_groups visits every group once", "[dag][visit_dag]") {
    const octree::Id root = octree::Id::root();
    const octree::Id first_child = root.child(0).value();
    const octree::Id second_child = root.child(1).value();
    const octree::Id grandchild = first_child.child(0).value();

    // The root's first group lists clusters of two batches.
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
    const TempDirectory directory("visit_dag");
    write_dag(directory.path(), {{root, root_node}, {first_child, first_child_node}, {second_child, second_child_node}, {grandchild, grandchild_node}});

    const ClusterRecord first_child_0{.id = {first_child, 0}, .error = first_child_node.groups[0].error, .lod_bounds = first_child_node.groups[0].bounds, .tight_bounds = first_child_node.clusters[0].bounds};
    const ClusterRecord second_child_0{.id = {second_child, 0}, .error = 0.0, .lod_bounds = second_child_node.clusters[0].bounds, .tight_bounds = second_child_node.clusters[0].bounds};
    const ClusterRecord second_child_1{.id = {second_child, 1}, .error = 0.0, .lod_bounds = second_child_node.clusters[1].bounds, .tight_bounds = second_child_node.clusters[1].bounds};
    const ClusterRecord grandchild_0{.id = {grandchild, 0}, .error = 0.0, .lod_bounds = grandchild_node.clusters[0].bounds, .tight_bounds = grandchild_node.clusters[0].bounds};

    const std::vector<GroupRecord> groups = collect_groups(directory.path());
    CHECK_THAT(groups, Catch::Matchers::UnorderedEquals(std::vector<GroupRecord>{
        {.id = {root, 0}, .error = root_node.groups[0].error, .lod_bounds = root_node.groups[0].bounds, .clusters = {first_child_0, second_child_0}},
        {.id = {root, 1}, .error = root_node.groups[1].error, .lod_bounds = root_node.groups[1].bounds, .clusters = {second_child_1}},
        {.id = {first_child, 0}, .error = first_child_node.groups[0].error, .lod_bounds = first_child_node.groups[0].bounds, .clusters = {grandchild_0}},
    }));
}

TEST_CASE("visit_groups stops when the visitor returns false", "[dag][visit_dag]") {
    const octree::Id root = octree::Id::root();
    const octree::Id child = root.children().value()[0];
    const radix::geometry::Aabb3d bounds{glm::dvec3{0.0}, glm::dvec3{1.0}};

    const TempDirectory directory("visit_dag");
    write_dag(directory.path(), {
        {root, dag::NodeMetadata{.clusters = {{.group_index = 0, .bounds = bounds}}, .groups = {{.children = {{child, 0}}, .error = 1.0, .bounds = bounds}, {.children = {{child, 1}}, .error = 1.0, .bounds = bounds}}}},
        {child, dag::NodeMetadata{.clusters = {{.group_index = std::nullopt, .bounds = bounds}, {.group_index = std::nullopt, .bounds = bounds}}, .groups = {}}},
    });
    const auto dag = REQUIRE_VALUE(octree::open_folder_indexed<dag::NodeMetadata>(directory.path()));
    dag::NodeMetadataCache cache(dag, UNBOUNDED_RESIDENT_BYTES);

    uint32_t visit_count = 0;
    const bool walked = REQUIRE_VALUE(dag::visit_groups([&](const dag::Group &) {
        visit_count++;
        return false;
    }, cache));

    CHECK_FALSE(walked);
    CHECK(visit_count == 1);
}

TEST_CASE("visit_root_clusters visits every cluster of the batches on the lowest level", "[dag][visit_dag]") {
    // Two batches on the lowest level, and one above it whose cluster a group of the first lists.
    const octree::Id first = octree::Id::root().child(0).value();
    const octree::Id second = octree::Id::root().child(1).value();
    const octree::Id above = first.child(0).value();

    const dag::NodeMetadata first_node{
        .clusters = {{.group_index = 0, .bounds = make_distinct_box(0)}, {.group_index = std::nullopt, .bounds = make_distinct_box(1)}},
        .groups = {{.children = {{above, 0}}, .error = 2.0, .bounds = make_distinct_box(2)}},
    };
    const dag::NodeMetadata second_node{
        .clusters = {{.group_index = std::nullopt, .bounds = make_distinct_box(3)}},
        .groups = {},
    };
    const dag::NodeMetadata above_node{
        .clusters = {{.group_index = std::nullopt, .bounds = make_distinct_box(4)}},
        .groups = {},
    };
    const TempDirectory directory("visit_dag");
    write_dag(directory.path(), {{first, first_node}, {second, second_node}, {above, above_node}});
    const auto dag = REQUIRE_VALUE(octree::open_folder_indexed<dag::NodeMetadata>(directory.path()));
    dag::NodeMetadataCache cache(dag, UNBOUNDED_RESIDENT_BYTES);

    std::vector<ClusterRecord> visited;
    const bool walked = REQUIRE_VALUE(dag::visit_root_clusters([&](const dag::Cluster &cluster) {
        visited.push_back(make_cluster_record(cluster));
    }, cache));

    CHECK(walked);
    CHECK_THAT(visited, Catch::Matchers::UnorderedEquals(std::vector<ClusterRecord>{
        {.id = {first, 0}, .error = first_node.groups[0].error, .lod_bounds = first_node.groups[0].bounds, .tight_bounds = first_node.clusters[0].bounds},
        {.id = {first, 1}, .error = 0.0, .lod_bounds = first_node.clusters[1].bounds, .tight_bounds = first_node.clusters[1].bounds},
        {.id = {second, 0}, .error = 0.0, .lod_bounds = second_node.clusters[0].bounds, .tight_bounds = second_node.clusters[0].bounds},
    }));
}

TEST_CASE("visit_root_clusters stops when the visitor returns false", "[dag][visit_dag]") {
    const octree::Id root = octree::Id::root();
    const radix::geometry::Aabb3d bounds{glm::dvec3{0.0}, glm::dvec3{1.0}};

    const TempDirectory directory("visit_dag");
    write_dag(directory.path(), {
        {root, dag::NodeMetadata{.clusters = {{.group_index = std::nullopt, .bounds = bounds}, {.group_index = std::nullopt, .bounds = bounds}}, .groups = {}}},
    });
    const auto dag = REQUIRE_VALUE(octree::open_folder_indexed<dag::NodeMetadata>(directory.path()));
    dag::NodeMetadataCache cache(dag, UNBOUNDED_RESIDENT_BYTES);

    uint32_t visit_count = 0;
    const bool walked = REQUIRE_VALUE(dag::visit_root_clusters([&](const dag::Cluster &) {
        visit_count++;
        return false;
    }, cache));

    CHECK_FALSE(walked);
    CHECK(visit_count == 1);
}
