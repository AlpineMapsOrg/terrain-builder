#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "node_header_helpers.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

#include "error/Error.h"
#include "error/Result.h"
#include "error/utils.h"
#include "io/Error.h"

#include "bvh/BatchWriter.h"
#include "bvh/NodeCache.h"
#include "bvh/Storage.h"
#include "bvh/Batch.h"
#include "bvh/NodeHeader.h"
#include "bvh/NodeView.h"
#include "bvh/Reference.h"
#include "bvh/traverse_tree.h"
#include "dag/SourceId.h"
#include "storage/batch_limits.h"

namespace {

enum class VisitError {
    Rejected,
};

std::string_view format_as(const VisitError) {
    return "visit rejected";
}

// A walk whose visitor can fail on its own as well as on a read.
using WalkError = alp::Error<io::Error, VisitError>;

}

TEST_CASE("the walk visits every node once, each parent before its children and children in stored order", "[bvh][traverse_tree]") {
    // Spread over one batch per node as well, with room for a single one in memory, so the order cannot come from the batches.
    const size_t max_batch_bytes = GENERATE(DEFAULT_MAX_BATCH_BYTES, size_t{1});

    const TempDirectory directory("traverse_tree");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes, max_batch_bytes);
    const bvh::Reference leaf_0 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_1 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_2 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_3 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference left = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{leaf_0, leaf_1}}));
    const bvh::Reference right = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{leaf_2, leaf_3}}));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{left, right}}));
    REQUIRE(writer.flush().has_value());
    REQUIRE(nodes.save_index().has_value());

    bvh::NodeCache cache(nodes, 1);
    std::vector<bvh::Reference> visited;
    const auto result = bvh::traverse_tree(cache, root, [&](const bvh::Reference &reference, const bvh::NodeView &) {
        visited.push_back(reference);
    });

    REQUIRE(result.value_or(false));
    CHECK(visited == std::vector{root, left, leaf_0, leaf_1, right, leaf_2, leaf_3});
}

TEST_CASE("a leaf root is visited alone, with its header and source","[bvh][traverse_tree]") {
    const bvh::NodeHeader leaf_header = make_node_header({3.0, 4.0}, {glm::dvec3{5.0}, glm::dvec3{6.0}});
    const dag::SourceId leaf_source = dag::GroupId{octree::Id::root(), 5};

    const TempDirectory directory("traverse_tree");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const auto leaf_result = writer.write_node(bvh::LeafInput{leaf_header, leaf_source});
    REQUIRE(leaf_result.has_value());
    REQUIRE(writer.flush().has_value());
    REQUIRE(nodes.save_index().has_value());

    bvh::NodeCache cache(nodes, 1);
    std::vector<bvh::Reference> visited;
    bvh::NodeHeader header;
    dag::SourceId source;
    const auto result = bvh::traverse_tree(cache, *leaf_result, [&](const bvh::Reference &reference, const bvh::NodeView &view) {
        visited.push_back(reference);
        header = view.header();
        source = view.source();
    });

    REQUIRE(result.value_or(false));
    CHECK(visited == std::vector{*leaf_result});
    CHECK(header == leaf_header);
    CHECK(source == leaf_source);
}

TEST_CASE("an inner node's view carries its header and children", "[bvh][traverse_tree]") {
    const bvh::NodeHeader root_header = make_node_header({1.0, 2.0}, {glm::dvec3{0.0}, glm::dvec3{4.0}});

    const TempDirectory directory("traverse_tree");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference first = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference second = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{root_header, std::array{first, second}}));
    REQUIRE(writer.flush().has_value());
    REQUIRE(nodes.save_index().has_value());

    bvh::NodeCache cache(nodes, 1);
    bvh::NodeHeader header;
    std::vector<bvh::Reference> children;
    const auto result = bvh::traverse_tree(cache, root, [&](const bvh::Reference &, const bvh::NodeView &view) {
        header = view.header();
        children.assign_range(view.children());
        return false;
    });

    REQUIRE(result.has_value());
    CHECK(header == root_header);
    CHECK(children == std::vector{first, second});
}

TEST_CASE("a visitor breaking off stops the walk", "[bvh][traverse_tree]") {
    const TempDirectory directory("traverse_tree");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference leaf_0 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_1 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_2 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_3 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference left = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{leaf_0, leaf_1}}));
    const bvh::Reference right = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{leaf_2, leaf_3}}));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{left, right}}));
    REQUIRE(writer.flush().has_value());
    REQUIRE(nodes.save_index().has_value());

    bvh::NodeCache cache(nodes, 1);
    uint32_t visits = 0;
    const auto result = bvh::traverse_tree(cache, root, [&](const bvh::Reference &, const bvh::NodeView &) {
        visits++;
        return visits < 3;
    });

    REQUIRE(result.has_value());
    CHECK_FALSE(result.value());
    CHECK(visits == 3);
}

TEST_CASE("a child whose batch cannot be read fails the walk", "[bvh][traverse_tree]") {
    const TempDirectory directory("traverse_tree");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));

    // Reserved but never written, so reading it must fail rather than end the walk quietly.
    const bvh::Batch missing = nodes.create_batch();

    bvh::BatchWriter writer(nodes);
    const auto root_result = writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{bvh::Reference{.batch = missing.index(), .node = 0}}});
    REQUIRE(root_result.has_value());
    REQUIRE(writer.flush().has_value());
    REQUIRE(nodes.save_index().has_value());

    bvh::NodeCache cache(nodes, 1);
    uint32_t visits = 0;
    const auto result = bvh::traverse_tree(cache, *root_result, [&](const bvh::Reference &, const bvh::NodeView &) {
        visits++;
    });

    CHECK_FALSE(result.has_value());
    CHECK(visits == 1);
}

TEST_CASE("a pruned node's children are skipped and the walk goes on with its siblings", "[bvh][traverse_tree]") {
    const TempDirectory directory("traverse_tree");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference leaf_0 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_1 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_2 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference leaf_3 = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference left = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{leaf_0, leaf_1}}));
    const bvh::Reference right = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{leaf_2, leaf_3}}));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{left, right}}));
    REQUIRE(writer.flush().has_value());
    REQUIRE(nodes.save_index().has_value());

    bvh::NodeCache cache(nodes, 1);
    std::vector<bvh::Reference> visited;
    const auto result = bvh::traverse_tree(cache, root, [&](const bvh::Reference &reference, const bvh::NodeView &) {
        visited.push_back(reference);
        if (reference == left) {
            return bvh::Control::Prune;
        } else {
            return bvh::Control::Continue;
        }
    });

    REQUIRE(result.value_or(false));
    CHECK(visited == std::vector{root, left, right, leaf_2, leaf_3});
}

TEST_CASE("a pruned leaf goes on like a continued one", "[bvh][traverse_tree]") {
    const TempDirectory directory("traverse_tree");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference first = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference second = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{first, second}}));
    REQUIRE(writer.flush().has_value());
    REQUIRE(nodes.save_index().has_value());

    bvh::NodeCache cache(nodes, 1);
    std::vector<bvh::Reference> visited;
    const auto result = bvh::traverse_tree(cache, root, [&](const bvh::Reference &reference, const bvh::NodeView &view) {
        visited.push_back(reference);
        if (view.is_leaf()) {
            return bvh::Control::Prune;
        } else {
            return bvh::Control::Continue;
        }
    });

    REQUIRE(result.value_or(false));
    CHECK(visited == std::vector{root, first, second});
}

TEST_CASE("a failing visitor fails the walk with its error and visits nothing after it", "[bvh][traverse_tree]") {
    const TempDirectory directory("traverse_tree");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));
    bvh::BatchWriter writer(nodes);
    const bvh::Reference first = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference second = REQUIRE_VALUE(writer.write_node(make_leaf_input()));
    const bvh::Reference root = REQUIRE_VALUE(writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{first, second}}));
    REQUIRE(writer.flush().has_value());
    REQUIRE(nodes.save_index().has_value());

    bvh::NodeCache cache(nodes, 1);
    std::vector<bvh::Reference> visited;
    const auto result = bvh::traverse_tree<WalkError>(cache, root, [&](const bvh::Reference &reference, const bvh::NodeView &) -> alp::Result<void, WalkError> {
        visited.push_back(reference);
        if (reference == first) {
            return alp::make_unexpected(VisitError::Rejected);
        }
        return {};
    });

    const WalkError error = REQUIRE_ERROR(result);
    CHECK(error.holds_code(VisitError::Rejected));
    CHECK(visited == std::vector{root, first});
}

TEST_CASE("a batch that cannot be read fails a walk with a wider error, holding the read's error", "[bvh][traverse_tree]") {
    const TempDirectory directory("traverse_tree");
    bvh::Storage nodes = REQUIRE_VALUE(bvh::Storage::create(directory.path()));

    // Reserved but never written, so reading it must fail rather than end the walk quietly.
    const bvh::Batch missing = nodes.create_batch();

    bvh::BatchWriter writer(nodes);
    const auto root_result = writer.write_node(bvh::InnerInput{bvh::NodeHeader{}, std::array{bvh::Reference{.batch = missing.index(), .node = 0}}});
    REQUIRE(root_result.has_value());
    REQUIRE(writer.flush().has_value());
    REQUIRE(nodes.save_index().has_value());

    bvh::NodeCache cache(nodes, 1);
    const auto result = bvh::traverse_tree<WalkError>(cache, *root_result, [&](const bvh::Reference &, const bvh::NodeView &) -> alp::Result<bool, WalkError> {
        return true;
    });

    const WalkError error = REQUIRE_ERROR(result);
    CHECK(error.holds_and<io::FileError>(&io::FileError::is_not_found));
}
