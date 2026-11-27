#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "dag_helpers.h"
#include "node_header_helpers.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <optional>
#include <vector>

#include "../dag_builder/storage.h"
#include "octree/Id.h"
#include "octree/storage/open.h"

#include "bvh/BatchWriter.h"
#include "bvh/NodeCache.h"
#include "bvh/NodeHeader.h"
#include "bvh/NodeInput.h"
#include "bvh/Reference.h"
#include "bvh/Storage.h"
#include "bvh/Batch.h"
#include "dag/NodeMetadataCache.h"
#include "dag/SourceId.h"
#include "view/Camera.h"
#include "view/CutRule.h"
#include "view/select_cut.h"

namespace {

constexpr size_t UNBOUNDED_RESIDENT_BYTES = std::numeric_limits<size_t>::max();

// The camera of every test.
const view::CameraOptions CAMERA{.eye = {0.0, 0.0, 0.0}, .forward = {1.0, 0.0, 0.0}, .up = {0.0, 0.0, 1.0}, .fov_y = 2.0 * std::atan(0.5), .aspect = 1.0, .z_near = 1.0};
// In front of the camera.
const bvh::Bounds IN_VIEW{{10.0, -1.0, -1.0}, {12.0, 1.0, 1.0}};
// Behind the camera.
const bvh::Bounds BEHIND{{-12.0, -1.0, -1.0}, {-10.0, 1.0, 1.0}};

// Two children in a batch that is reserved but never written, so reading them fails.
std::array<bvh::Reference, 2> make_unreadable_children(bvh::Storage &nodes) {
    const bvh::Batch missing = nodes.create_batch();
    return {bvh::Reference{.batch = missing.index(), .node = 0}, bvh::Reference{.batch = missing.index(), .node = 1}};
}

// Selects the groups for `CAMERA` from the tree at `root` in `nodes`.
view::GroupSelection select_camera_groups(bvh::Storage &nodes, const bvh::Reference &root, const view::CutRule &rule = view::CutRule{.threshold = 0.5}) {
    const view::Camera camera = REQUIRE_VALUE(view::Camera::create(CAMERA));
    bvh::NodeCache cache(nodes, 1);
    return REQUIRE_VALUE(view::select_groups(cache, root, camera, rule));
}

// Selects the clusters of `groups` for `CAMERA` from the dag at `path`.
std::vector<dag::ClusterId> select_camera_clusters(const std::filesystem::path &path, const std::vector<dag::SourceId> &groups, const view::CutRule &rule = view::CutRule{.threshold = 0.5}) {
    const view::Camera camera = REQUIRE_VALUE(view::Camera::create(CAMERA));
    const auto dag = REQUIRE_VALUE(dag::storage::open_metadata(path));
    dag::NodeMetadataCache cache(dag, UNBOUNDED_RESIDENT_BYTES);
    return REQUIRE_VALUE(view::select_clusters(groups, cache, camera, rule));
}

// A dag whose group lists two clusters of a child batch, each produced by a group of its own with `errors` and `lod_bounds`, and bounded tightly by `tight_bounds`.
// Returns the listing group.
dag::GroupId write_listing_group(const std::filesystem::path &path, const std::array<double, 2> &errors, const std::array<bvh::Bounds, 2> &lod_bounds, const std::array<bvh::Bounds, 2> &tight_bounds) {
    const octree::Id root = octree::Id::root();
    const octree::Id child = root.child(0).value();
    write_dag(path, {
        {root, dag::NodeMetadata{.clusters = {}, .groups = {{.children = {{child, 0}, {child, 1}}, .error = 2000.0, .bounds = IN_VIEW}}}},
        {child, dag::NodeMetadata{
            .clusters = {{.group_index = 0, .bounds = tight_bounds[0]}, {.group_index = 1, .bounds = tight_bounds[1]}},
            .groups = {{.children = {}, .error = errors[0], .bounds = lod_bounds[0]}, {.children = {}, .error = errors[1], .bounds = lod_bounds[1]}},
        }},
    });
    return dag::GroupId{root, 0};
}

}

TEST_CASE("select_groups skips a subtree out of view", "[view][select_cut]") {
    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const auto unreadable = make_unreadable_children(nodes);
    bvh::BatchWriter writer(nodes);
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_node_header({4.0, 6.0}, BEHIND), unreadable}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_groups(nodes, root) == view::GroupSelection{.groups = {}, .inner_nodes_tested = 1, .leaves_tested = 0});
}

TEST_CASE("select_groups skips a subtree whose largest parent error fits", "[view][select_cut]") {
    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const auto unreadable = make_unreadable_children(nodes);
    bvh::BatchWriter writer(nodes);
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_node_header({0.0, 0.001}, IN_VIEW), unreadable}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_groups(nodes, root) == view::GroupSelection{.groups = {}, .inner_nodes_tested = 1, .leaves_tested = 0});
}

TEST_CASE("select_groups skips a leaf whose errors all are too coarse", "[view][select_cut]") {
    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::LeafInput{make_node_header({1000.0, 2000.0}, IN_VIEW), dag::GroupId{octree::Id::root(), 0}}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_groups(nodes, root) == view::GroupSelection{.groups = {}, .inner_nodes_tested = 0, .leaves_tested = 1});
}

TEST_CASE("select_groups skips a leaf whose errors all fit", "[view][select_cut]") {
    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::LeafInput{make_node_header({0.0, 0.001}, IN_VIEW), dag::GroupId{octree::Id::root(), 0}}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_groups(nodes, root) == view::GroupSelection{.groups = {}, .inner_nodes_tested = 0, .leaves_tested = 1});
}

TEST_CASE("select_groups selects a leaf the cut crosses", "[view][select_cut]") {
    const dag::SourceId group = dag::GroupId{octree::Id::root(), 0};

    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::LeafInput{make_node_header({0.001, 2000.0}, IN_VIEW), group}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_groups(nodes, root) == view::GroupSelection{.groups = {group}, .inner_nodes_tested = 0, .leaves_tested = 1});
}

TEST_CASE("select_groups selects a root leaf whose error fits", "[view][select_cut]") {
    const dag::SourceId cluster = dag::ClusterId{octree::Id::root(), 0};

    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::LeafInput{make_node_header({0.001, bvh::ROOT_PARENT_ERROR}, IN_VIEW), cluster}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_groups(nodes, root) == view::GroupSelection{.groups = {cluster}, .inner_nodes_tested = 0, .leaves_tested = 1});
}

TEST_CASE("select_groups descends into a subtree out of view without culling", "[view][select_cut]") {
    const dag::SourceId group = dag::GroupId{octree::Id::root(), 0};

    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference leaf = REQUIRE_VALUE(writer.write_node(bvh::LeafInput{make_node_header({0.001, 2000.0}, BEHIND), group}));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_node_header({0.001, 2000.0}, BEHIND), std::array{leaf}}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_groups(nodes, root, view::CutRule{.threshold = 0.5, .cull = false}) == view::GroupSelection{.groups = {group}, .inner_nodes_tested = 1, .leaves_tested = 1});
}

TEST_CASE("select_clusters selects the clusters whose own error fits", "[view][select_cut]") {
    const TempDirectory directory("select_cut");
    const dag::GroupId group = write_listing_group(directory.path() / "dag", {0.001, 1000.0}, {IN_VIEW, IN_VIEW}, {IN_VIEW, IN_VIEW});

    CHECK(select_camera_clusters(directory.path() / "dag", {group}) == std::vector<dag::ClusterId>{{octree::Id::root().child(0).value(), 0}});
}

TEST_CASE("select_clusters selects a cluster no group lists whose error fits", "[view][select_cut]") {
    const octree::Id root = octree::Id::root();
    const TempDirectory directory("select_cut");
    write_dag(directory.path() / "dag", {
        {root, dag::NodeMetadata{.clusters = {{.group_index = 0, .bounds = IN_VIEW}}, .groups = {{.children = {}, .error = 0.001, .bounds = IN_VIEW}}}},
    });

    CHECK(select_camera_clusters(directory.path() / "dag", {dag::ClusterId{root, 0}}) == std::vector<dag::ClusterId>{{root, 0}});
}

TEST_CASE("select_clusters leaves out a cluster whose tight bounds are out of view", "[view][select_cut]") {
    const TempDirectory directory("select_cut");
    // Both LOD bounds are in view, so testing them instead would draw both.
    const dag::GroupId group = write_listing_group(directory.path() / "dag", {0.001, 0.001}, {IN_VIEW, IN_VIEW}, {IN_VIEW, BEHIND});

    CHECK(select_camera_clusters(directory.path() / "dag", {group}) == std::vector<dag::ClusterId>{{octree::Id::root().child(0).value(), 0}});
}

TEST_CASE("select_clusters keeps a cluster out of view without culling", "[view][select_cut]") {
    const octree::Id child = octree::Id::root().child(0).value();
    const TempDirectory directory("select_cut");
    const dag::GroupId group = write_listing_group(directory.path() / "dag", {0.001, 0.001}, {IN_VIEW, IN_VIEW}, {IN_VIEW, BEHIND});

    CHECK(select_camera_clusters(directory.path() / "dag", {group}, view::CutRule{.threshold = 0.5, .cull = false}) == std::vector<dag::ClusterId>{{child, 0}, {child, 1}});
}

TEST_CASE("select_clusters measures each cluster's error from its own LOD bounds", "[view][select_cut]") {
    // An error at a distance shows as their ratio, so it fits if it is at most half the distance.
    // The cluster's LOD bounds lie 100 away, where its error of 20 fits, but its tight bounds lie 10 away, where it does not.
    const bvh::Bounds far{{100.0, -1.0, -1.0}, {102.0, 1.0, 1.0}};

    const TempDirectory directory("select_cut");
    const dag::GroupId group = write_listing_group(directory.path() / "dag", {20.0, 1000.0}, {far, IN_VIEW}, {IN_VIEW, IN_VIEW});

    CHECK(select_camera_clusters(directory.path() / "dag", {group}) == std::vector<dag::ClusterId>{{octree::Id::root().child(0).value(), 0}});
}

TEST_CASE("select_clusters fails if a batch it needs is not in the dag", "[view][select_cut]") {
    const octree::Id root = octree::Id::root();
    const TempDirectory directory("select_cut");
    write_dag(directory.path() / "dag", {{root, dag::NodeMetadata{.clusters = {}, .groups = {}}}});
    const view::Camera camera = REQUIRE_VALUE(view::Camera::create(CAMERA));
    const auto dag = REQUIRE_VALUE(dag::storage::open_metadata(directory.path() / "dag"));
    dag::NodeMetadataCache cache(dag, UNBOUNDED_RESIDENT_BYTES);
    const std::vector<dag::SourceId> groups{dag::GroupId{root.child(0).value(), 0}};

    REQUIRE_ERROR(view::select_clusters(groups, cache, camera, view::CutRule{.threshold = 0.5}));
}
