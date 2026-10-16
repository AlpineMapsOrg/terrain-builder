#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "node_header_helpers.h"

#include <array>
#include <cmath>
#include <vector>

#include "octree/Id.h"

#include "bvh/BatchWriter.h"
#include "bvh/NodeCache.h"
#include "bvh/NodeHeader.h"
#include "bvh/NodeInput.h"
#include "bvh/Reference.h"
#include "bvh/Storage.h"
#include "bvh/Batch.h"
#include "dag/ClusterInfo.h"
#include "view/Camera.h"
#include "view/CutRule.h"
#include "view/select_cut.h"

namespace {

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

// Selects the cut for `CAMERA` from the tree at `root` in `nodes`.
view::Selection select_camera_cut(bvh::Storage &nodes, const bvh::Reference &root, const view::CutRule &rule = view::CutRule{.threshold = 0.5}) {
    const view::Camera camera = REQUIRE_VALUE(view::Camera::create(CAMERA));
    bvh::NodeCache cache(nodes, 1);
    return REQUIRE_VALUE(view::select_cut(cache, root, camera, rule));
}

}

TEST_CASE("select_cut skips a subtree out of view", "[view][select_cut]") {
    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const auto unreadable = make_unreadable_children(nodes);
    bvh::BatchWriter writer(nodes);
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_node_header({4.0, 6.0}, BEHIND), unreadable}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_cut(nodes, root) == view::Selection{.clusters = {}, .inner_nodes_tested = 1, .leaves_tested = 0});
}

TEST_CASE("select_cut skips a subtree whose largest parent error fits", "[view][select_cut]") {
    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    const auto unreadable = make_unreadable_children(nodes);
    bvh::BatchWriter writer(nodes);
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_node_header({0.0, 0.001}, IN_VIEW), unreadable}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_cut(nodes, root) == view::Selection{.clusters = {}, .inner_nodes_tested = 1, .leaves_tested = 0});
}

TEST_CASE("select_cut skips a leaf whose errors all are too coarse", "[view][select_cut]") {
    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    // Its cluster would be drawn, so only skipping the leaf keeps it off the cut.
    const std::array clusters{dag::ClusterInfo{.id = {octree::Id::root(), 0}, .error = 0.001, .lod_bounds = IN_VIEW, .tight_bounds = IN_VIEW}};
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::LeafInput{make_node_header({1000.0, 2000.0}, IN_VIEW), clusters}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_cut(nodes, root) == view::Selection{.clusters = {}, .inner_nodes_tested = 0, .leaves_tested = 1});
}

TEST_CASE("select_cut skips a leaf whose errors all fit", "[view][select_cut]") {
    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    // Its cluster would be drawn, so only skipping the leaf keeps it off the cut.
    const std::array clusters{dag::ClusterInfo{.id = {octree::Id::root(), 0}, .error = 0.0, .lod_bounds = IN_VIEW, .tight_bounds = IN_VIEW}};
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::LeafInput{make_node_header({0.0, 0.001}, IN_VIEW), clusters}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_cut(nodes, root) == view::Selection{.clusters = {}, .inner_nodes_tested = 0, .leaves_tested = 1});
}

TEST_CASE("select_cut draws the clusters of a leaf whose own error fits", "[view][select_cut]") {
    const octree::Id node = octree::Id::root();

    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const std::array clusters{
        dag::ClusterInfo{.id = {node, 0}, .error = 0.001, .lod_bounds = IN_VIEW, .tight_bounds = IN_VIEW},
        dag::ClusterInfo{.id = {node, 1}, .error = 1000.0, .lod_bounds = IN_VIEW, .tight_bounds = IN_VIEW},
    };
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::LeafInput{make_node_header({0.001, 2000.0}, IN_VIEW), clusters}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_cut(nodes, root).clusters == std::vector<dag::Id>{{node, 0}});
}

TEST_CASE("select_cut draws the clusters of a root leaf that fit", "[view][select_cut]") {
    const octree::Id node = octree::Id::root();

    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const std::array clusters{dag::ClusterInfo{.id = {node, 0}, .error = 0.001, .lod_bounds = IN_VIEW, .tight_bounds = IN_VIEW}};
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::LeafInput{make_node_header({0.001, bvh::ROOT_PARENT_ERROR}, IN_VIEW), clusters}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_cut(nodes, root).clusters == std::vector<dag::Id>{{node, 0}});
}

TEST_CASE("select_cut leaves out a cluster whose tight bounds are out of view", "[view][select_cut]") {
    const octree::Id node = octree::Id::root();

    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    // Both LOD bounds are in view, so testing them instead would draw both.
    const std::array clusters{
        dag::ClusterInfo{.id = {node, 0}, .error = 0.001, .lod_bounds = IN_VIEW, .tight_bounds = IN_VIEW},
        dag::ClusterInfo{.id = {node, 1}, .error = 0.001, .lod_bounds = IN_VIEW, .tight_bounds = BEHIND},
    };
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::LeafInput{make_node_header({0.001, 2000.0}, IN_VIEW), clusters}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_cut(nodes, root).clusters == std::vector<dag::Id>{{node, 0}});
}

TEST_CASE("select_cut keeps a cluster out of view without culling", "[view][select_cut]") {
    const octree::Id node = octree::Id::root();

    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const std::array clusters{
        dag::ClusterInfo{.id = {node, 0}, .error = 0.001, .lod_bounds = IN_VIEW, .tight_bounds = IN_VIEW},
        dag::ClusterInfo{.id = {node, 1}, .error = 0.001, .lod_bounds = IN_VIEW, .tight_bounds = BEHIND},
    };
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::LeafInput{make_node_header({0.001, 2000.0}, IN_VIEW), clusters}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_cut(nodes, root, view::CutRule{.threshold = 0.5, .cull = false}).clusters == std::vector<dag::Id>{{node, 0}, {node, 1}});
}

TEST_CASE("select_cut descends into a subtree out of view without culling", "[view][select_cut]") {
    const octree::Id node = octree::Id::root();

    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const std::array clusters{dag::ClusterInfo{.id = {node, 0}, .error = 0.001, .lod_bounds = BEHIND, .tight_bounds = BEHIND}};
    const bvh::Reference leaf = REQUIRE_VALUE(writer.write_node(bvh::LeafInput{make_node_header({0.001, 2000.0}, BEHIND), clusters}));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{make_node_header({0.001, 2000.0}, BEHIND), std::array{leaf}}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_cut(nodes, root, view::CutRule{.threshold = 0.5, .cull = false}) == view::Selection{.clusters = {{node, 0}}, .inner_nodes_tested = 1, .leaves_tested = 1});
}

TEST_CASE("select_cut measures each cluster's error from its own LOD bounds", "[view][select_cut]") {
    const octree::Id node = octree::Id::root();
    // An error at a distance shows as their ratio, so it fits if it is at most half the distance.
    // The cluster's LOD bounds lie 100 away, where its error of 20 fits, but the leaf's reach to 10, where it does not.
    const bvh::Bounds far{{100.0, -1.0, -1.0}, {102.0, 1.0, 1.0}};
    const bvh::Bounds in_view_to_far{{10.0, -1.0, -1.0}, {102.0, 1.0, 1.0}};

    const TempDirectory directory("select_cut");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const std::array clusters{dag::ClusterInfo{.id = {node, 0}, .error = 20.0, .lod_bounds = far, .tight_bounds = IN_VIEW}};
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::LeafInput{make_node_header({20.0, 2000.0}, in_view_to_far, IN_VIEW), clusters}));
    REQUIRE_VALUE(writer.flush());
    REQUIRE_VALUE(nodes.save_index());

    CHECK(select_camera_cut(nodes, root).clusters == std::vector<dag::Id>{{node, 0}});
}
