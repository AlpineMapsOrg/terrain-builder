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

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/std.h>
#include <glm/glm.hpp>
#include <libassert/assert.hpp>
#include <meshoptimizer.h>
#include <opencv2/core.hpp>
#include <radix/geometry.h>

#include "codec.h"

#include "cli_support/log.h"
#include "cluster.h"
#include "dag_node.h"
#include "dag_validate.h"
#include "enumerate.h"
#include "error/Context.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/OpenCvError.h"
#include "io/WriteMode.h"
#include "io/alloc.h"
#include "io/envelope.h"
#include "io/image/Format.h"
#include "io/limits.h"
#include "io/mat.h"
#include "io/radix_serialization.h"
#include "meshopt/encode.h"
#include "path_utils.h"
#include "store/envelope_utils.h"

namespace dag::codec {
namespace {

store::CodecReadError to_codec_read_error(io::mat::DecodeError error) {
    return std::move(error).map<io::mat::CorruptData>(store::CorruptData{}).map<io::OpenCvError>(store::Unknown{});
}

store::CodecWriteError to_codec_write_error(io::mat::EncodeError error) {
    return std::move(error).map<io::OpenCvError>(store::Unknown{});
}

namespace format {

namespace v1 {

struct OctreeId {
    uint8_t level;
    uint64_t index;
};

struct Id {
    OctreeId source_batch;
    uint32_t cluster_index;
};

struct Texture {
    std::vector<uint8_t> jpeg;
};

struct Cluster {
    uint64_t triangle_count;
    uint64_t vertex_count;
    uint64_t uv_count;
    std::vector<uint8_t> encoded_triangles;
    std::vector<uint8_t> encoded_vertex_indices;
    std::vector<uint8_t> encoded_uvs;
    uint32_t id;
    uint32_t texture_id;
    double absolute_error;
};

struct Clustering {
    uint64_t vertex_count;
    std::vector<uint8_t> encoded_positions;
    std::vector<Cluster> clusters;
    std::vector<Texture> textures;
};

struct Group {
    std::vector<Id> children;
    double error;
    radix::geometry::Aabb3d bounds;
};

struct NodeMetadata {
    std::vector<uint32_t> group_assignment;
    std::vector<Group> groups;
};

} // namespace v1

namespace v2 {

struct NodeMetadata {
    std::vector<uint32_t> group_assignment;
    std::vector<uint32_t> triangle_counts;
    std::vector<v1::Group> groups;

    static NodeMetadata from_previous(v1::NodeMetadata previous) {
        LOG_WARN_ONCE("Reading v1 node metadata which didn't include triangle counts, filling with 0");
        const size_t cluster_count = previous.group_assignment.size();
        return {
            .group_assignment = std::move(previous.group_assignment),
            .triangle_counts = std::vector<uint32_t>(cluster_count),
            .groups = std::move(previous.groups),
        };
    }
};

} // namespace v2

using MetadataSchema = io::envelope::PayloadSchema<"dag.NodeMetadata", io::envelope::Version<1, v1::NodeMetadata>, io::envelope::Version<2, v2::NodeMetadata>>;
using ClusteringSchema = io::envelope::PayloadSchema<"dag.Clustering", io::envelope::Version<1, v1::Clustering>>;
using NodeMetadata = MetadataSchema::Latest;
using Clustering = ClusteringSchema::Latest;

v1::OctreeId encode_octree_id(const octree::Id& id) {
    return {
        .level = id.level(),
        .index = id.index_on_level()
    };
}

alp::Result<octree::Id, store::CorruptData> decode_octree_id(const v1::OctreeId& id) {
    const std::optional<octree::Id> id_opt = octree::Id::try_make(id.level, id.index);
    if (!id_opt.has_value()) {
        return alp::make_error(store::CorruptData{}, fmt::format("level {} has no octree node {}", id.level, id.index));
    }
    return *id_opt;
}

// The texture id the file stores for a cluster without a texture.
constexpr uint32_t null_texture_id = std::numeric_limits<uint32_t>::max();

uint32_t encode_texture_id(const std::optional<uint32_t> texture_id) {
    ASSERT(texture_id != null_texture_id, "the texture id is the one the file reserves for no texture");
    return texture_id.value_or(null_texture_id);
}

std::optional<uint32_t> decode_texture_id(const uint32_t texture_id) {
    if (texture_id == null_texture_id) {
        return std::nullopt;
    } else {
        return texture_id;
    }
}

NodeMetadata encode_metadata(const dag::NodeMetadata& metadata) {
    std::vector<v1::Group> groups;
    groups.reserve(metadata.groups.size());
    for (const dag::Group& group : metadata.groups) {
        std::vector<v1::Id> children;
        children.reserve(group.children.size());
        for (const dag::Id& child : group.children) {
            children.push_back({ .source_batch = encode_octree_id(child.source_batch), .cluster_index = child.cluster_index });
        }
        groups.push_back({ .children = std::move(children), .error = group.error, .bounds = group.bounds });
    }

    return {
        .group_assignment = metadata.group_assignment,
        .triangle_counts = metadata.triangle_counts,
        .groups = std::move(groups),
    };
}

alp::Result<dag::NodeMetadata, store::CorruptData> decode_metadata(NodeMetadata metadata) {
    std::vector<dag::Group> groups;
    groups.reserve(metadata.groups.size());
    for (const auto [group_index, group] : enumerate(metadata.groups)) {
        const auto group_context = alp::make_context("decoding group {}", group_index);
        std::vector<dag::Id> children;
        children.reserve(group.children.size());
        for (const v1::Id& child : group.children) {
            ALP_TRY_CONTEXT(const octree::Id source_batch, decode_octree_id(child.source_batch), group_context);
            children.push_back({ source_batch, child.cluster_index });
        }
        groups.push_back({ .children = std::move(children), .error = group.error, .bounds = group.bounds });
    }

    return dag::NodeMetadata{
        .group_assignment = std::move(metadata.group_assignment),
        .triangle_counts = std::move(metadata.triangle_counts),
        .groups = std::move(groups),
    };
}

v1::Cluster encode_cluster(const ::Cluster& cluster) {
    std::vector<uint8_t> encoded_triangles = meshopt::encode_index_buffer(cluster.local_triangles);
    std::vector<uint8_t> encoded_vertex_indices = meshopt::encode_vertex_buffer(cluster.vertex_indices);
    std::vector<uint8_t> encoded_uvs = meshopt::encode_vertex_buffer(cluster.uvs);

    return {
        .triangle_count = cluster.local_triangles.size(),
        .vertex_count = cluster.vertex_indices.size(),
        .uv_count = cluster.uvs.size(),
        .encoded_triangles = std::move(encoded_triangles),
        .encoded_vertex_indices = std::move(encoded_vertex_indices),
        .encoded_uvs = std::move(encoded_uvs),
        .id = cluster.id,
        .texture_id = encode_texture_id(cluster.texture_id),
        .absolute_error = cluster.absolute_error,
    };
}

alp::Result<Clustering, store::CodecWriteError> encode_clustering(const ::Clustering& clustering) {
    std::vector<uint8_t> encoded_positions = meshopt::encode_vertex_buffer(clustering.positions);

    std::vector<v1::Cluster> clusters;
    clusters.reserve(clustering.clusters.size());
    for (const ::Cluster& cluster : clustering.clusters) {
        clusters.push_back(encode_cluster(cluster));
    }

    std::vector<v1::Texture> textures;
    textures.reserve(clustering.textures.size());
    for (const auto [texture_index, texture] : enumerate(clustering.textures)) {
        const auto texture_context = alp::make_context("encoding texture {}", texture_index);
        auto jpeg_result = io::mat::encode(texture, io::image::Format::Jpeg).transform_error(to_codec_write_error);
        ALP_TRY_CONTEXT(io::bytes::vector jpeg, std::move(jpeg_result), texture_context);
        textures.push_back({ std::move(jpeg) });
    }

    return Clustering{
        .vertex_count = clustering.positions.size(),
        .encoded_positions = std::move(encoded_positions),
        .clusters = std::move(clusters),
        .textures = std::move(textures),
    };
}

alp::Result<void, io::BufferLimitExceeded> check_decoded_size(const Clustering& clustering) {
    io::ByteBudget budget;
    ALP_TRY(budget.try_claim<glm::dvec3>(clustering.vertex_count));
    for (const v1::Cluster& cluster : clustering.clusters) {
        ALP_TRY(budget.try_claim<glm::uvec3>(cluster.triangle_count));
        ALP_TRY(budget.try_claim<uint32_t>(cluster.vertex_count));
        ALP_TRY(budget.try_claim<glm::dvec2>(cluster.uv_count));
    }
    return {};
}

template <typename T>
alp::Result<std::vector<T>, store::CodecReadError> decode_vertex_buffer(const uint64_t count, const std::vector<uint8_t>& encoded) {
    ALP_TRY(std::vector<T> vertices, io::try_make_vector<T>(count));
    auto decode_result = meshopt::try_decode_vertex_buffer(std::span(vertices), encoded);
    if (!decode_result.has_value()) {
        return alp::make_error(store::CorruptData{}, std::move(decode_result).error());
    }
    return vertices;
}

alp::Result<std::vector<glm::uvec3>, store::CodecReadError> decode_triangles(const uint64_t count, const std::vector<uint8_t>& encoded) {
    ALP_TRY(std::vector<glm::uvec3> triangles, io::try_make_vector<glm::uvec3>(count));
    auto decode_result = meshopt::try_decode_index_buffer(std::span(triangles), encoded);
    if (!decode_result.has_value()) {
        return alp::make_error(store::CorruptData{}, std::move(decode_result).error());
    }
    return triangles;
}

alp::Result<::Cluster, store::CodecReadError> decode_cluster(const v1::Cluster& encoded) {
    ALP_TRY_CONTEXT(std::vector<glm::uvec3> triangles, decode_triangles(encoded.triangle_count, encoded.encoded_triangles), "decoding the triangles");
    ALP_TRY_CONTEXT(std::vector<uint32_t> vertex_indices, decode_vertex_buffer<uint32_t>(encoded.vertex_count, encoded.encoded_vertex_indices), "decoding the vertex indices");
    ALP_TRY_CONTEXT(std::vector<glm::dvec2> uvs, decode_vertex_buffer<glm::dvec2>(encoded.uv_count, encoded.encoded_uvs), "decoding the UVs");

    return ::Cluster{
        .id = encoded.id,
        .vertex_indices = std::move(vertex_indices),
        .local_triangles = std::move(triangles),
        .uvs = std::move(uvs),
        .texture_id = decode_texture_id(encoded.texture_id),
        .absolute_error = encoded.absolute_error,
    };
}

alp::Result<::Clustering, store::CodecReadError> decode_clustering(Clustering clustering) {
    const auto size_context = alp::make_context("checking that the decoded clustering fits in {} bytes", io::max_buffer_size);
    ALP_TRY_CONTEXT(check_decoded_size(clustering), size_context);
    ALP_TRY_CONTEXT(std::vector<glm::dvec3> positions, decode_vertex_buffer<glm::dvec3>(clustering.vertex_count, clustering.encoded_positions), "decoding the positions");

    std::vector<::Cluster> clusters;
    clusters.reserve(clustering.clusters.size());
    for (const auto [cluster_index, encoded] : enumerate(clustering.clusters)) {
        const auto cluster_context = alp::make_context("decoding cluster {}", cluster_index);
        ALP_TRY_CONTEXT(::Cluster cluster, decode_cluster(encoded), cluster_context);
        clusters.push_back(std::move(cluster));
    }

    std::vector<cv::Mat> textures;
    textures.reserve(clustering.textures.size());
    for (const auto [texture_index, texture] : enumerate(clustering.textures)) {
        const auto texture_context = alp::make_context("decoding texture {}", texture_index);
        auto image_result = io::mat::decode(texture.jpeg).transform_error(to_codec_read_error);
        ALP_TRY_CONTEXT(cv::Mat image, std::move(image_result), texture_context);
        textures.push_back(std::move(image));
    }

    return ::Clustering{
        .positions = std::move(positions),
        .clusters = std::move(clusters),
        .textures = TextureSet(std::move(textures)),
    };
}

} // namespace format

// A node's hash concatenates the .dag and .dagmeta envelope hashes, so that Metadata can verify the .dagmeta file on its own.
constexpr size_t file_hash_size = 8;

struct FileHashes {
    std::optional<store::PayloadHashView> clustering;
    std::optional<store::PayloadHashView> metadata;
};

alp::Result<FileHashes, store::CorruptData> split_node_hash(const std::optional<store::PayloadHashView> node_hash) {
    if (!node_hash.has_value()) {
        return FileHashes{};
    }
    if (node_hash->size() != 2 * file_hash_size) {
        return alp::make_error(store::CorruptData{}, fmt::format("the expected node hash has {} bytes, but a DAG node hash has {}", node_hash->size(), 2 * file_hash_size));
    }
    return FileHashes{
        .clustering = node_hash->first(file_hash_size),
        .metadata = node_hash->subspan(file_hash_size),
    };
}

alp::Result<dag::NodeMetadata, store::CodecReadError> read_metadata(const std::filesystem::path& path, const std::optional<store::PayloadHashView> expected_hash) {
    auto payload_result = io::envelope::read<format::MetadataSchema>(path, { .expected_hash = expected_hash }).transform_error(store::to_codec_read_error);
    ALP_TRY(format::NodeMetadata payload, std::move(payload_result));
    ALP_TRY(dag::NodeMetadata metadata, format::decode_metadata(std::move(payload)));

    auto validation_result = alp::map_error<dag::InconsistentMetadata>(dag::validate(metadata), store::CorruptData{});
    ALP_TRY(std::move(validation_result));
    return metadata;
}

} // namespace

std::vector<std::filesystem::path> ClusterBatch::paths(const std::filesystem::path& node_path) const {
    return {
        paths::with_added_extension(node_path, clustering_extension),
        paths::with_added_extension(node_path, metadata_extension),
    };
}

alp::Result<dag::ClusterBatch, store::CodecReadError> ClusterBatch::read(const std::filesystem::path& node_path, const std::optional<store::PayloadHashView> expected_hash) const {
    const auto context = alp::make_context("reading a DAG batch from {}", node_path);
    const std::vector<std::filesystem::path> node_paths = this->paths(node_path);
    ALP_TRY_CONTEXT(const FileHashes expected_file_hashes, split_node_hash(expected_hash), context);

    auto clustering_payload_result = io::envelope::read<format::ClusteringSchema>(node_paths[0], { .expected_hash = expected_file_hashes.clustering }).transform_error(store::to_codec_read_error);
    ALP_TRY_CONTEXT(format::Clustering clustering_payload, std::move(clustering_payload_result), context);
    ALP_TRY_CONTEXT(dag::NodeMetadata metadata, read_metadata(node_paths[1], expected_file_hashes.metadata), context);
    ALP_TRY_CONTEXT(::Clustering clustering, format::decode_clustering(std::move(clustering_payload)), context);

    auto validation_result = alp::map_error<dag::InconsistentBatch>(dag::validate(metadata, clustering), store::CorruptData{});
    ALP_TRY_CONTEXT(std::move(validation_result), context);
    return dag::ClusterBatch{
        .metadata = std::move(metadata),
        .clustering = std::move(clustering),
    };
}

alp::Result<store::PayloadHash, store::CodecWriteError> ClusterBatch::write(const std::filesystem::path& node_path, const dag::ClusterBatch& batch) const {
    dag::assert_valid(batch);
    const auto context = alp::make_context("writing a DAG batch to {}", node_path);
    const std::vector<std::filesystem::path> node_paths = this->paths(node_path);

    ALP_TRY_CONTEXT(const format::Clustering clustering, format::encode_clustering(batch.clustering), context);
    const format::NodeMetadata metadata = format::encode_metadata(batch.metadata);

    auto clustering_hash_result = io::envelope::write<format::ClusteringSchema>(clustering, node_paths[0], { .mode = io::WriteMode::CreateNew }).transform_error(store::to_codec_write_error);
    ALP_TRY_CONTEXT(store::PayloadHash clustering_hash, std::move(clustering_hash_result), context);
    auto metadata_hash_result = io::envelope::write<format::MetadataSchema>(metadata, node_paths[1], { .mode = io::WriteMode::CreateNew }).transform_error(store::to_codec_write_error);
    ALP_TRY_CONTEXT(const store::PayloadHash metadata_hash, std::move(metadata_hash_result), context);

    ASSERT(clustering_hash.size() == file_hash_size && metadata_hash.size() == file_hash_size);
    store::PayloadHash node_hash = std::move(clustering_hash);
    node_hash.insert(node_hash.end(), metadata_hash.begin(), metadata_hash.end());
    return node_hash;
}

std::vector<std::filesystem::path> Metadata::paths(const std::filesystem::path& node_path) const {
    return { paths::with_added_extension(node_path, metadata_extension) };
}

alp::Result<dag::NodeMetadata, store::CodecReadError> Metadata::read(const std::filesystem::path& node_path, const std::optional<store::PayloadHashView> expected_hash) const {
    const auto context = alp::make_context("reading DAG metadata from {}", node_path);
    const std::filesystem::path path = this->paths(node_path).front();
    ALP_TRY_CONTEXT(const FileHashes expected_file_hashes, split_node_hash(expected_hash), context);
    ALP_TRY_CONTEXT(dag::NodeMetadata metadata, read_metadata(path, expected_file_hashes.metadata), context);
    return metadata;
}

} // namespace dag::codec
