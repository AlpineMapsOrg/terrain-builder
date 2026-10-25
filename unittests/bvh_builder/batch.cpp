#include "../catch2_helpers.h"
#include "node_header_helpers.h"

#include <vector>

#include <glm/glm.hpp>

#include "octree/Id.h"
#include "range_utils.h"

#include "bvh/Batch.h"
#include "bvh/NodeHeader.h"
#include "bvh/NodeInput.h"
#include "bvh/NodeView.h"
#include "bvh/Reference.h"
#include "dag/SourceId.h"

TEST_CASE("Batch::add_node stores an inner node's children and a leaf's source, which resolve gives back", "[bvh][Batch]") {
    const bvh::NodeHeader inner_header = make_node_header({1.0, 2.0}, {glm::dvec3{0.0}, glm::dvec3{1.0}});
    const bvh::NodeHeader leaf_header = make_node_header({3.0, 4.0}, {glm::dvec3{2.0}, glm::dvec3{3.0}});
    const std::vector<bvh::Reference> children{{.batch = 1, .node = 2}, {.batch = 3, .node = 4}, {.batch = 5, .node = 6}};
    const dag::SourceId first_source = dag::GroupId{octree::Id::root(), 7};
    const dag::SourceId second_source = dag::ClusterId{octree::Id::root(), 9};

    bvh::Batch batch(0);
    const bvh::NodeIndex first_leaf = batch.add_node(bvh::LeafInput{leaf_header, first_source});
    const bvh::NodeIndex inner = batch.add_node(bvh::InnerInput{inner_header, children});
    const bvh::NodeIndex second_leaf = batch.add_node(bvh::LeafInput{leaf_header, second_source});

    const bvh::NodeView first_leaf_view = batch.resolve(batch.node(first_leaf));
    const bvh::NodeView inner_view = batch.resolve(batch.node(inner));
    const bvh::NodeView second_leaf_view = batch.resolve(batch.node(second_leaf));

    CHECK(first_leaf_view.header() == leaf_header);
    CHECK(first_leaf_view.source() == first_source);
    CHECK(inner_view.header() == inner_header);
    CHECK(to_vector(inner_view.children()) == children);
    CHECK(second_leaf_view.header() == leaf_header);
    CHECK(second_leaf_view.source() == second_source);
}
