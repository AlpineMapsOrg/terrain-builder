/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include <filesystem>
#include <future>
#include <system_error>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "../assertion_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"
#include "codec.h"
#include "common_error_codes.h"
#include "dag_node.h"
#include "io/bytes.h"
#include "io/envelope.h"
#include "io/filesystem.h"

namespace {

dag::ClusterBatch sample_batch() {
    dag::ClusterBatch batch;
    batch.metadata.clusters = { { .group_index = 0, .bounds = { { 0.0, 0.0, 0.0 }, { 1.0, 1.0, 0.0 } } } };
    batch.metadata.groups = { {
        .children = { { octree::Id::root().child(4).value(), 3 } },
        .error = 12.5,
        .bounds = { { 0.5, 1.5, 2.5 }, { 3.5, 4.5, 5.5 } },
    } };
    batch.clustering.positions = {
        { 0.0, 0.0, 0.0 },
        { 1.0, 0.0, 0.0 },
        { 0.0, 1.0, 0.0 },
    };
    batch.clustering.clusters = { {
        .id = 17,
        .vertex_indices = { 0, 1, 2 },
        .local_triangles = { { 0, 1, 2 } },
        .uvs = {},
        .texture_id = std::nullopt,
        .absolute_error = 0.0,
    } };
    return batch;
}

} // namespace

TEST_CASE("runtime DAG envelope codec is reentrant", "[dag_builder][codec]") {
    const dag::ClusterBatch batch = sample_batch();

    TempDirectory output;
    dag::codec::ClusterBatch codec;
    std::vector<std::future<bool>> operations;
    for (int index = 0; index < 8; ++index) {
        operations.push_back(std::async(std::launch::async, [&codec, &batch, &output, index] {
            const std::filesystem::path path(output.path() / std::to_string(index) / "node");
            if (!codec.write(path, batch).has_value()) {
                return false;
            }
            const auto loaded = codec.read(path);
            return loaded.has_value() && loaded->metadata.clusters == batch.metadata.clusters;
        }));
    }
    for (auto& operation : operations) {
        REQUIRE(operation.get());
    }
}

TEST_CASE("read gives back the metadata write stored", "[dag_builder][codec]") {
    TempDirectory output;
    dag::codec::ClusterBatch codec;
    const std::filesystem::path node_path(output.path() / "node");
    const dag::ClusterBatch batch = sample_batch();
    REQUIRE_VALUE(codec.write(node_path, batch));

    const dag::ClusterBatch read_batch = REQUIRE_VALUE(codec.read(node_path));

    CHECK(read_batch.metadata == batch.metadata);
}

TEST_CASE("read gives back the metadata of a leaf cluster", "[dag_builder][codec]") {
    TempDirectory output;
    dag::codec::ClusterBatch codec;
    const std::filesystem::path node_path(output.path() / "node");
    dag::ClusterBatch batch = sample_batch();
    batch.metadata.clusters.front().group_index = std::nullopt;
    batch.metadata.groups.clear();
    REQUIRE_VALUE(codec.write(node_path, batch));

    const dag::ClusterBatch read_batch = REQUIRE_VALUE(codec.read(node_path));

    CHECK(read_batch.metadata == batch.metadata);
}

TEST_CASE("DAG codec rejects invalid batches and inconsistent files", "[dag_builder][codec]") {
    TempDirectory output;
    dag::codec::ClusterBatch codec;
    const std::filesystem::path node_path(output.path() / "node");
    const auto node_paths = codec.paths(node_path);

    SECTION("write") {
        auto batch = sample_batch();
        batch.metadata.clusters.clear();

        CHECK_ASSERTS(codec.write(node_path, batch));
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(node_paths[0])));
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(node_paths[1])));
    }

    SECTION("write invalid metadata") {
        auto batch = sample_batch();
        batch.metadata.clusters.front().group_index = 1;

        CHECK_ASSERTS(codec.write(node_path, batch));
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(node_paths[0])));
        CHECK_FALSE(REQUIRE_VALUE(io::filesystem::exists(node_paths[1])));
    }

    SECTION("read") {
        REQUIRE_VALUE(codec.write(node_path, sample_batch()));
        const std::filesystem::path empty_node_path(output.path() / "empty-node");
        const auto empty_node_paths = codec.paths(empty_node_path);
        REQUIRE_VALUE(codec.write(empty_node_path, dag::ClusterBatch {}));
        REQUIRE_VALUE(io::filesystem::copy_file(empty_node_paths[1], node_paths[1], io::filesystem::CopyMode::Overwrite));

        const auto error = REQUIRE_ERROR(codec.read(node_path));

        CHECK(error.holds<store::CorruptData>());
    }
}

TEST_CASE("DAG codec never replaces existing files", "[dag_builder][codec]") {
    TempDirectory output;
    dag::codec::ClusterBatch codec;
    const std::filesystem::path node_path(output.path() / "node");
    const auto node_paths = codec.paths(node_path);
    REQUIRE_VALUE(codec.write(node_path, sample_batch()));
    const io::bytes::vector original_data = REQUIRE_VALUE(io::bytes::read(node_paths[0]));
    const io::bytes::vector original_metadata = REQUIRE_VALUE(io::bytes::read(node_paths[1]));

    auto batch = sample_batch();
    batch.metadata.groups.front().error = 99.0;
    const auto error = REQUIRE_ERROR(codec.write(node_path, batch));
    CHECK(error.holds_code(std::errc::file_exists));
    const io::bytes::vector data = REQUIRE_VALUE(io::bytes::read(node_paths[0]));
    const io::bytes::vector metadata = REQUIRE_VALUE(io::bytes::read(node_paths[1]));
    CHECK(data == original_data);
    CHECK(metadata == original_metadata);
}

TEST_CASE("DAG codecs store batches in two files and read metadata from one", "[dag_builder][codec]") {
    CHECK(dag::codec::ClusterBatch{}.paths("node") == std::vector<std::filesystem::path> { "node.dag", "node.dagmeta" });
    CHECK(dag::codec::Metadata{}.paths("node") == std::vector<std::filesystem::path> { "node.dagmeta" });

    const auto error = REQUIRE_ERROR(dag::codec::Metadata{}.write("node", dag::NodeMetadata {}));

    CHECK(error.holds<Unsupported>());
}

TEST_CASE("DAG node hashes concatenate the clustering and metadata hashes", "[dag_builder][codec]") {
    TempDirectory output;
    dag::codec::ClusterBatch codec;
    const std::filesystem::path node_path(output.path() / "node");
    const store::PayloadHash hash = REQUIRE_VALUE(codec.write(node_path, sample_batch()));
    REQUIRE(hash.size() == 16);
    const auto node_paths = codec.paths(node_path);
    const io::envelope::Header clustering_header = REQUIRE_VALUE(io::envelope::read_header(node_paths[0]));
    const io::envelope::Header metadata_header = REQUIRE_VALUE(io::envelope::read_header(node_paths[1]));
    CHECK(std::vector(hash.begin(), hash.begin() + 8) == clustering_header.hash);
    CHECK(std::vector(hash.begin() + 8, hash.end()) == metadata_header.hash);
}

TEST_CASE("DAG reads verify each half of an expected node hash", "[dag_builder][codec]") {
    TempDirectory output;
    dag::codec::ClusterBatch codec;
    const std::filesystem::path node_path(output.path() / "node");
    const store::PayloadHash hash = REQUIRE_VALUE(codec.write(node_path, sample_batch()));
    REQUIRE_VALUE(codec.read(node_path, hash));

    auto expected = hash;
    SECTION("clustering half") {
        expected.front() += 1;
    }
    SECTION("metadata half") {
        expected.back() += 1;
    }
    SECTION("another length") {
        expected.resize(8);
    }
    const auto error = REQUIRE_ERROR(codec.read(node_path, expected));
    CHECK(error.holds<store::CorruptData>());
}

TEST_CASE("DAG metadata reads verify only the metadata half", "[dag_builder][codec]") {
    TempDirectory output;
    dag::codec::ClusterBatch codec;
    dag::codec::Metadata metadata_codec;
    CHECK(metadata_codec.supports_hashing());
    const std::filesystem::path node_path(output.path() / "node");
    const store::PayloadHash hash = REQUIRE_VALUE(codec.write(node_path, sample_batch()));
    REQUIRE(REQUIRE_VALUE(io::filesystem::remove(codec.paths(node_path).front())));

    REQUIRE_VALUE(metadata_codec.read(node_path, hash));
    auto wrong_clustering = hash;
    wrong_clustering.front() += 1;
    REQUIRE_VALUE(metadata_codec.read(node_path, wrong_clustering));

    auto wrong_metadata = hash;
    wrong_metadata.back() += 1;
    const auto wrong_metadata_error = REQUIRE_ERROR(metadata_codec.read(node_path, wrong_metadata));
    CHECK(wrong_metadata_error.holds<store::CorruptData>());
    const auto short_hash_error = REQUIRE_ERROR(metadata_codec.read(node_path, store::PayloadHash(8)));
    CHECK(short_hash_error.holds<store::CorruptData>());
}
