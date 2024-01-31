#ifndef IO_H
#define IO_H

#include <filesystem>
#include <optional>
#include <vector>
#include <span>

#define CGLTF_IMPLEMENTATION
#define CGLTF_WRITE_IMPLEMENTATION
// #define CGLTF_VALIDATE_ENABLE_ASSERTS
#include <cgltf_write.h>
#include <fmt/core.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <tl/expected.hpp>

#include "non_copyable.h"
#include "terrain_mesh.h"

#define GL_NEAREST 0x2600
#define GL_LINEAR 0x2601
#define GL_NEAREST_MIPMAP_NEAREST 0x2700
#define GL_LINEAR_MIPMAP_NEAREST 0x2701
#define GL_NEAREST_MIPMAP_LINEAR 0x2702
#define GL_LINEAR_MIPMAP_LINEAR 0x2703
#define GL_TEXTURE_MAG_FILTER 0x2800
#define GL_TEXTURE_MIN_FILTER 0x2801
#define GL_REPEAT 0x2901
#define GL_CLAMP_TO_EDGE 0x812F
#define GL_MIRRORED_REPEAT 0x8370

namespace io {
class RawGltfMesh : public NonCopyable {
public:
    cgltf_data *data;

    ~RawGltfMesh() {
        if (this->data != nullptr) {
            cgltf_free(this->data);
        }
    }

    // Move constructor
    RawGltfMesh(RawGltfMesh &&other) {
        this->data = other.data;
        other.data = nullptr;
    }

    // Move assignment operator
    RawGltfMesh &operator=(RawGltfMesh &&other) {
        if (this != &other) {
            this->data = other.data;
            other.data = nullptr;
        }
        return *this;
    }

    static tl::expected<RawGltfMesh, cgltf_result> load_from_path(const std::filesystem::path &path) {
        cgltf_options options = {};
        cgltf_data *data = NULL;
        cgltf_result result = cgltf_parse_file(&options, path.string().c_str(), &data);
        if (result != cgltf_result::cgltf_result_success) {
            return tl::unexpected(result);
        }
        return RawGltfMesh(data);
    }

private:
    RawGltfMesh(cgltf_data *data)
        : data(data) {}
};

namespace {
cgltf_attribute *find_attribute_with_type(cgltf_attribute *attributes, size_t attribute_count, cgltf_attribute_type type) {
    for (unsigned int i = 0; i < attribute_count; i++) {
        cgltf_attribute *attribute = &attributes[i];
        if (attribute->type == type) {
            return attribute;
        }
    }

    return nullptr;
}
} // namespace


inline cv::Mat read_texture_from_encoded_bytes(std::span<const uint8_t> buffer) {
    cv::Mat raw_data = cv::Mat(1, buffer.size(), CV_8UC1, const_cast<uint8_t*>(buffer.data()));
    cv::Mat mat = cv::imdecode(raw_data, cv::IMREAD_UNCHANGED);
    mat.convertTo(mat, CV_32FC3);
    return mat;
}
inline void write_texture_to_encoded_buffer(const cv::Mat& image, std::vector<uint8_t>& buffer) {
    cv::Mat converted;
    image.convertTo(converted, CV_8UC3);
    cv::imencode(".png", image, buffer);
}
inline std::vector<uint8_t> write_texture_to_encoded_buffer(const cv::Mat& image) {
    std::vector<uint8_t> buffer;
    write_texture_to_encoded_buffer(image, buffer);
    return buffer;
}

TerrainMesh load_mesh_from_raw(const RawGltfMesh &raw) {
    const cgltf_data &data = *raw.data;
    assert(data.file_type == cgltf_file_type::cgltf_file_type_glb);

    assert(data.meshes_count == 1);
    cgltf_mesh &mesh = data.meshes[0];

    assert(mesh.primitives_count == 1);
    cgltf_primitive &mesh_primitive = mesh.primitives[0];
    assert(mesh_primitive.type == cgltf_primitive_type::cgltf_primitive_type_triangles);

    cgltf_attribute *position_attr = find_attribute_with_type(mesh_primitive.attributes, mesh_primitive.attributes_count, cgltf_attribute_type_position);
    cgltf_attribute *uv_attr = find_attribute_with_type(mesh_primitive.attributes, mesh_primitive.attributes_count, cgltf_attribute_type_texcoord);
    assert(position_attr != nullptr);
    assert(uv_attr != nullptr);

    assert(data.buffers_count > 0);
    cgltf_buffer &first_buffer = data.buffers[0];
    if (first_buffer.data == nullptr) {
        first_buffer.data = const_cast<void *>(data.bin);
        first_buffer.size = data.bin_size;
    }

    cgltf_accessor &index_accessor = *mesh_primitive.indices;
    std::vector<glm::uvec3> indices;
    indices.resize(index_accessor.count / 3);
    cgltf_accessor_unpack_indices(&index_accessor, reinterpret_cast<unsigned int *>(indices.data()), cgltf_component_size(index_accessor.component_type), indices.size() * 3);

    cgltf_accessor &position_accessor = *position_attr->data;
    cgltf_accessor &uv_accessor = *uv_attr->data;
    assert(position_accessor.buffer_view == uv_accessor.buffer_view);
    std::vector<glm::vec3> positions;
    positions.resize(position_accessor.count);
    cgltf_accessor_unpack_floats(&position_accessor, reinterpret_cast<float *>(positions.data()), positions.size() * 3);
    std::vector<glm::vec2> uvs;
    uvs.resize(uv_accessor.count);
    cgltf_accessor_unpack_floats(&uv_accessor, reinterpret_cast<float *>(uvs.data()), uvs.size() * 2);

    assert(data.scenes_count == 1);
    assert(data.scenes[0].nodes_count == 1);
    cgltf_node &root_node = *data.scenes[0].nodes[0];
    assert(root_node.has_translation);
    assert(root_node.children_count == 1);
    cgltf_node &intermediate_node = *root_node.children[0];
    assert(intermediate_node.has_translation);
    assert(intermediate_node.children_count == 1);
    cgltf_node &mesh_node = *intermediate_node.children[0];
    assert(mesh_node.has_translation);
    assert(mesh_node.children_count == 0);
    glm::dvec3 offset =
        glm::dvec3(root_node.translation[0], root_node.translation[1], root_node.translation[2]) +
        glm::dvec3(intermediate_node.translation[0], intermediate_node.translation[1], intermediate_node.translation[2]) +
        glm::dvec3(mesh_node.translation[0], mesh_node.translation[1], mesh_node.translation[2]);

    cgltf_material &material = *mesh_primitive.material;
    assert(material.has_pbr_metallic_roughness);
    assert(material.pbr_metallic_roughness.base_color_texture.texture != nullptr);
    cgltf_texture &albedo_texture = *material.pbr_metallic_roughness.base_color_texture.texture;
    cgltf_image &albedo_image = *albedo_texture.image;

    const std::span<const uint8_t> raw_texture {cgltf_buffer_view_data(albedo_image.buffer_view), albedo_image.buffer_view->size};
    cv::Mat texture = read_texture_from_encoded_bytes(raw_texture);

    std::vector<glm::dvec3> positionsd;
    positionsd.resize(positions.size());
    for (unsigned int i = 0; i < positionsd.size(); i++) {
        positionsd[i] = glm::dvec3(positions[i]) + offset;
    }

    std::vector<glm::dvec2> uvsd;
    uvsd.resize(uvs.size());
    for (unsigned int i = 0; i < uvsd.size(); i++) {
        uvsd[i] = glm::dvec2(uvs[i]);
    }

    return TerrainMesh(indices, positionsd, uvsd, texture);
}

enum class LoadMeshErrorKind {
    FileNotFound,
    InvalidFormat,
    OutOfMemory
};

class LoadMeshError {
public:
    LoadMeshError() = default;
    constexpr LoadMeshError(LoadMeshErrorKind kind)
        : kind(kind) {}

    operator LoadMeshErrorKind() const {
        return this->kind;
    }
    constexpr bool operator==(LoadMeshError other) const {
        return this->kind == other.kind;
    }
    constexpr bool operator!=(LoadMeshError other) const {
        return this->kind != other.kind;
    }

    std::string description() const {
        switch (kind) {
        case LoadMeshErrorKind::FileNotFound:
            return "file not found";
        case LoadMeshErrorKind::InvalidFormat:
            return "invalid file format";
        case LoadMeshErrorKind::OutOfMemory:
            return "out of memory";
        default:
            return "undefined error";
        }
    }

private:
    LoadMeshErrorKind kind;
};

namespace {
LoadMeshError map_cgltf_error(cgltf_result result) {
    switch (result) {
    case cgltf_result::cgltf_result_data_too_short:
    case cgltf_result::cgltf_result_unknown_format:
    case cgltf_result::cgltf_result_invalid_json:
    case cgltf_result::cgltf_result_invalid_gltf:
    case cgltf_result::cgltf_result_legacy_gltf:
        return LoadMeshErrorKind::InvalidFormat;
    case cgltf_result::cgltf_result_file_not_found:
    case cgltf_result::cgltf_result_io_error:
        return LoadMeshErrorKind::FileNotFound;
    case cgltf_result::cgltf_result_out_of_memory:
        return LoadMeshErrorKind::OutOfMemory;
    default:
        assert(false);
        break;
    }
}
} // namespace

tl::expected<TerrainMesh, LoadMeshError> load_mesh_from_path(const std::filesystem::path &path) {
    tl::expected<RawGltfMesh, cgltf_result> raw_mesh = RawGltfMesh::load_from_path(path);
    if (!raw_mesh) {
        return tl::unexpected(map_cgltf_error(raw_mesh.error()));
    }
    TerrainMesh mesh = load_mesh_from_raw(*raw_mesh);
    return mesh;
}

enum class SaveMeshErrorKind {
};

class SaveMeshError {
public:
    SaveMeshError() = default;
    constexpr SaveMeshError(SaveMeshErrorKind kind)
        : kind(kind) {}

    operator SaveMeshErrorKind() const {
        return this->kind;
    }
    constexpr bool operator==(SaveMeshError other) const {
        return this->kind == other.kind;
    }
    constexpr bool operator!=(SaveMeshError other) const {
        return this->kind != other.kind;
    }

    std::string description() const {
        switch (kind) {
        default:
            return "undefined error";
        }
    }

private:
    SaveMeshErrorKind kind;
};

namespace {
/// Calculates the size of the data of a vector in bytes.
template <typename T>
size_t vectorsizeof(const typename std::vector<T> &vec) {
    return sizeof(T) * vec.size();
}

/// Encodes the data at the given pointer into base64.
// from https://github.com/syoyo/tinygltf/blob/5e8a7fd602af22aa9619ccd3baeaeeaff0ecb6f3/tiny_gltf.h#L2297
std::string base64_encode(unsigned char const *bytes_to_encode,
                          unsigned int in_len) {
    std::string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    const char *base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";

    while (in_len--) {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] =
                ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] =
                ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for (i = 0; (i < 4); i++)
                ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i) {
        for (j = i; j < 3; j++)
            char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] =
            ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] =
            ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

        for (j = 0; (j < i + 1); j++)
            ret += base64_chars[char_array_4[j]];

        while ((i++ < 3))
            ret += '=';
    }

    return ret;
}

/// Encodes the given data as a base64 data uri.
std::string data_uri_encode(unsigned char const *bytes_to_encode, unsigned int in_len) {
    return "data:application/octet-stream;base64," + base64_encode(bytes_to_encode, in_len);
}

inline size_t align(std::size_t alignment, size_t offset) noexcept {
    const size_t aligned = (offset - 1u + alignment) & -alignment;
    return aligned;
}
} // namespace

/// Saves the mesh as a .gltf or .glb file at the given path.
void save_mesh_as_gltf2(const TerrainMesh &terrain_mesh, const std::filesystem::path &path, const std::unordered_map<std::string, std::string> extra_metadata = {}) {
    // ********************* Preprocessing ********************* //

    // Calculate the average vertex position for later normalization.
    const size_t vertex_count = terrain_mesh.positions.size();
    glm::dvec3 average_position(0, 0, 0);
    for (size_t i = 0; i < vertex_count; i++) {
        average_position += terrain_mesh.positions[i] / static_cast<double>(vertex_count);
    }

    // Create vertex data vector from positions and uvs.
    // We also normalize vertex position by extracting their average position and storing the offsets.
    // This is dont to preserve our double accuracy, as gltf cannot store them directly.
    std::vector<float> vertices;
    vertices.reserve((vectorsizeof(terrain_mesh.positions) + vectorsizeof(terrain_mesh.uvs)) / sizeof(float));
    glm::vec3 max_position(std::numeric_limits<float>::min());
    glm::vec3 min_position(std::numeric_limits<float>::max());
    for (size_t i = 0; i < vertex_count; i++) {
        const glm::vec3 normalized_position = terrain_mesh.positions[i] - average_position;

        vertices.push_back(normalized_position.x);
        vertices.push_back(normalized_position.y);
        vertices.push_back(normalized_position.z);
        vertices.push_back(terrain_mesh.uvs[i].x);
        vertices.push_back(terrain_mesh.uvs[i].y);

        max_position = glm::max(max_position, normalized_position);
        min_position = glm::min(min_position, normalized_position);
    }

    // Encode the texture as jpeg data.
    const bool has_texture = terrain_mesh.has_texture();
    std::vector<uint8_t> texture_bytes;
    if (has_texture) {
        texture_bytes = write_texture_to_encoded_buffer(terrain_mesh.texture.value());
    }

    // Create a single buffer that holds all binary data (indices, vertices, textures)
    // We need to do this because only a single buffer can be written as a binary blob in .glb files.
    const size_t index_data_offset = align(sizeof(glm::uvec3), 0);
    const size_t index_data_byte_count = vectorsizeof(terrain_mesh.triangles);
    const size_t index_data_end = index_data_offset + index_data_byte_count;
    const size_t vertex_data_offset = align(sizeof(glm::vec3), index_data_end);
    const size_t vertex_data_byte_count = vectorsizeof(vertices);
    const size_t vertex_data_end = vertex_data_offset + vertex_data_byte_count;
    const size_t texture_data_offset = align(sizeof(uint8_t), vertex_data_end);
    const size_t texture_data_byte_count = vectorsizeof(texture_bytes);
    const size_t texture_data_end = texture_data_offset + texture_data_byte_count;

    std::vector<uint8_t> buffer_data;
    buffer_data.resize(texture_data_end);
    memcpy(buffer_data.data() + index_data_offset, terrain_mesh.triangles.data(), index_data_byte_count);
    memcpy(buffer_data.data() + vertex_data_offset, vertices.data(), vertex_data_byte_count);
    memcpy(buffer_data.data() + texture_data_offset, texture_bytes.data(), texture_data_byte_count);

    const bool binary_output = path.extension() == ".glb";

    // ********************* Create GLTF data structure ********************* //

    // Initialize a GLTF data structure
    cgltf_data data = {};
    std::string version = "2.0\0";
    data.asset.version = version.data();
    std::string generator = "alpinite\0";
    data.asset.generator = generator.data();

    std::array<cgltf_buffer, 1> buffers;

    // Create a gltf buffer to hold vertex data, index data and the texture.
    cgltf_buffer &buffer = buffers[0] = {};
    buffer.size = buffer_data.size();
    buffer.data = buffer_data.data();
    std::string buffer_data_encoded;
    if (binary_output) {
        // The binary blob at the end of the file will be used as the contents of the first buffer if it does not have an uri defined.
        data.bin = buffer_data.data();
        data.bin_size = buffer_data.size();
    } else {
        buffer_data_encoded = data_uri_encode(buffer_data.data(), buffer_data.size());
        buffer.uri = buffer_data_encoded.data();
    }

    // Create buffer views for each of types of data in the buffer.
    std::array<cgltf_buffer_view, 3> buffer_views;

    cgltf_buffer_view &index_buffer_view = buffer_views[0] = {};
    index_buffer_view.buffer = &buffer;
    index_buffer_view.offset = index_data_offset;
    index_buffer_view.size = index_data_byte_count;
    index_buffer_view.stride = 0;
    index_buffer_view.type = cgltf_buffer_view_type_indices;

    cgltf_buffer_view &vertex_buffer_view = buffer_views[1] = {};
    vertex_buffer_view.buffer = &buffer;
    vertex_buffer_view.offset = vertex_data_offset;
    vertex_buffer_view.size = vertex_data_byte_count;
    vertex_buffer_view.stride = 5 * sizeof(float);
    vertex_buffer_view.type = cgltf_buffer_view_type_vertices;

    cgltf_buffer_view &texture_buffer_view = buffer_views[2] = {};
    texture_buffer_view.buffer = &buffer;
    texture_buffer_view.offset = texture_data_offset;
    texture_buffer_view.size = texture_data_byte_count;
    texture_buffer_view.stride = 0;

    // Create accessors describing the layout of data in the buffer views.
    std::array<cgltf_accessor, 3> accessors;

    // Create an accessor for indices.
    cgltf_accessor &index_accessor = accessors[0] = {};
    index_accessor.buffer_view = &index_buffer_view;
    index_accessor.type = cgltf_type_scalar;
    index_accessor.component_type = cgltf_component_type_r_32u;
    index_accessor.count = terrain_mesh.face_count() * 3;
    index_accessor.has_min = true;
    index_accessor.min[0] = static_cast<cgltf_float>(0.0);
    index_accessor.has_max = true;
    index_accessor.max[0] = static_cast<cgltf_float>(vertex_count - 1);

    // Create an accessor for vertex positions
    cgltf_accessor &position_accessor = accessors[1] = {};
    position_accessor.buffer_view = &vertex_buffer_view;
    position_accessor.component_type = cgltf_component_type_r_32f;
    position_accessor.type = cgltf_type_vec3;
    position_accessor.offset = 0 * sizeof(float);
    position_accessor.count = vertex_count;
    // We need the min and max as some viewers otherwise refuse to open the file.
    position_accessor.has_min = true;
    position_accessor.has_max = true;
    std::copy(glm::value_ptr(min_position), glm::value_ptr(min_position) + min_position.length(), position_accessor.min);
    std::copy(glm::value_ptr(max_position), glm::value_ptr(max_position) + max_position.length(), position_accessor.max);

    // Create an accessor for vertex uvs
    cgltf_accessor &uv_accessor = accessors[2] = {};
    uv_accessor.buffer_view = &vertex_buffer_view;
    uv_accessor.component_type = cgltf_component_type_r_32f;
    uv_accessor.type = cgltf_type_vec2;
    uv_accessor.offset = 3 * sizeof(float);
    uv_accessor.count = vertex_count;
    uv_accessor.has_min = true;
    uv_accessor.has_max = true;
    std::fill(uv_accessor.min, uv_accessor.min + 2, 0);
    std::fill(uv_accessor.max, uv_accessor.max + 2, 1);

    // Create a mesh primitive.
    std::array<cgltf_attribute, 2> primitive_attributes;
    cgltf_attribute &position_attribute = primitive_attributes[0] = {};
    std::string position_attribute_name = "POSITION\0";
    position_attribute.name = position_attribute_name.data();
    position_attribute.type = cgltf_attribute_type::cgltf_attribute_type_position;
    position_attribute.index = 0;
    position_attribute.data = &position_accessor;
    cgltf_attribute &uv_attribute = primitive_attributes[1] = {};
    std::string uv_attribute_name = "TEXCOORD_0\0";
    uv_attribute.name = uv_attribute_name.data();
    uv_attribute.type = cgltf_attribute_type::cgltf_attribute_type_texcoord;
    uv_attribute.index = 0;
    uv_attribute.data = &uv_accessor;

    // Create a gltf texture
    std::array<cgltf_image, 1> images;
    cgltf_image &image = images[0] = {};
    image.buffer_view = &texture_buffer_view;
    std::string image_mime_type = "image/jpeg\0";
    image.mime_type = image_mime_type.data();

    std::array<cgltf_sampler, 1> samplers;
    cgltf_sampler &sampler = samplers[0] = {};
    sampler.min_filter = GL_LINEAR_MIPMAP_LINEAR;
    sampler.mag_filter = GL_LINEAR;
    sampler.wrap_s = GL_CLAMP_TO_EDGE;
    sampler.wrap_t = GL_CLAMP_TO_EDGE;

    std::array<cgltf_texture, 1> textures;
    cgltf_texture &texture = textures[0] = {};
    texture.image = &image;
    texture.sampler = &sampler;

    // Create a material
    std::array<cgltf_material, 1> materials;
    cgltf_material &material = materials[0] = {};
    material.has_pbr_metallic_roughness = true;
    material.pbr_metallic_roughness.base_color_factor[0] = 1.0f;
    material.pbr_metallic_roughness.base_color_factor[1] = 0.9f;
    material.pbr_metallic_roughness.base_color_factor[2] = 0.9f;
    material.pbr_metallic_roughness.base_color_factor[3] = 1.0f;
    material.pbr_metallic_roughness.roughness_factor = 1;
    if (has_texture) {
        material.pbr_metallic_roughness.base_color_texture.texture = &texture;
    }
    material.double_sided = true;

    // Build the primitive for the mesh
    std::array<cgltf_primitive, 1> primitives;
    cgltf_primitive &primitive = primitives[0] = {};
    primitive.type = cgltf_primitive_type_triangles;
    primitive.indices = &index_accessor;
    primitive.attributes_count = primitive_attributes.size();
    primitive.attributes = primitive_attributes.data();
    primitive.material = &material;

    // Build the actual mesh
    std::array<cgltf_mesh, 1> meshes;
    cgltf_mesh &mesh = meshes[0] = {};
    mesh.primitives_count = 1;
    mesh.primitives = &primitive;

    // Create the node hierachy.
    // We create parent nodes to offset the position by the average calculate above.
    // We need multiple parents to ensure that we dont lose our double precision accurary.
    std::array<cgltf_node, 3> nodes;
    cgltf_node &mesh_node = nodes[2] = {};
    mesh_node.has_translation = true;
    mesh_node.mesh = &mesh;

    cgltf_node &parent_node = nodes[1] = {};
    std::array<cgltf_node *, 1> parent_node_children = {&mesh_node};
    parent_node.children_count = parent_node_children.size();
    parent_node.children = parent_node_children.data();
    parent_node.has_translation = true;

    cgltf_node &parent_parent_node = nodes[0] = {};
    std::array<cgltf_node *, 1> parent_parent_node_children = {&parent_node};
    parent_parent_node.children_count = parent_parent_node_children.size();
    parent_parent_node.children = parent_parent_node_children.data();
    parent_parent_node.has_translation = true;

    const glm::vec3 parent_parent_offset(average_position);
    const glm::dvec3 parent_parent_offset_error = glm::dvec3(parent_parent_offset) - average_position;
    const glm::vec3 parent_offset(-parent_parent_offset_error);
    const glm::dvec3 parent_offset_error = glm::dvec3(parent_offset) + glm::dvec3(parent_parent_offset) - average_position;
    const glm::vec3 mesh_offset(-parent_offset_error);
    std::copy(glm::value_ptr(parent_parent_offset), glm::value_ptr(parent_parent_offset) + parent_parent_offset.length(), parent_node.translation);
    std::copy(glm::value_ptr(parent_offset), glm::value_ptr(parent_offset) + parent_offset.length(), parent_node.translation);
    std::copy(glm::value_ptr(mesh_offset), glm::value_ptr(mesh_offset) + mesh_offset.length(), mesh_node.translation);
    const glm::dvec3 full_error = (glm::dvec3(parent_parent_offset) + glm::dvec3(parent_offset) + glm::dvec3(mesh_offset)) - average_position;
    assert(glm::length(full_error) == 0);
    // Create a scene
    std::array<cgltf_scene, 1> scenes;
    cgltf_scene &scene = scenes[0] = {};
    std::array<cgltf_node *, 1> scene_nodes = {&parent_parent_node};
    scene.nodes_count = scene_nodes.size();
    scene.nodes = scene_nodes.data();

    // Set up data references
    data.meshes_count = meshes.size();
    data.meshes = meshes.data();
    data.nodes_count = nodes.size();
    data.nodes = nodes.data();
    data.scenes_count = scenes.size();
    data.scenes = scenes.data();
    data.buffers_count = buffers.size();
    data.buffers = buffers.data();
    data.buffer_views_count = buffer_views.size();
    data.buffer_views = buffer_views.data();
    data.accessors_count = accessors.size();
    data.accessors = accessors.data();
    data.materials_count = materials.size();
    data.materials = materials.data();
    if (has_texture) {
        data.textures_count = textures.size();
        data.textures = textures.data();
        data.images_count = images.size();
        data.images = images.data();
        data.samplers_count = samplers.size();
        data.samplers = samplers.data();
    } else {
        data.buffer_views_count = 2;
    }

    // Set up extra metadata
    std::string extras_str;
    if (!extra_metadata.empty()) {
        std::stringstream extras = {};
        extras << "{";
        for (auto const &[key, val] : extra_metadata) {
            extras << "\n";
            extras << "    ";
            extras << "\"" << key << "\"";
            extras << ": ";
            extras << val;
            extras << ",";
        }
        extras.seekp(-1, std::ios_base::end); // remove last ","
        extras << "\n  }";
        extras << "\0";
        extras_str = extras.str();
        data.extras.data = extras_str.data();
    }

    // ********************* Save the GLTF data to a file ********************* //
    std::filesystem::create_directories(path.parent_path());
    cgltf_options options;
    if (binary_output) {
        options.type = cgltf_file_type_glb;
    }
    if (cgltf_write_file(&options, path.c_str(), &data) != cgltf_result_success) {
        throw std::runtime_error("Failed to save GLTF file");
    }
}

/*
/// Saves the mesh as a .gltf or .glb file at the given path.
void save_mesh_as_gltf(TerrainMesh &terrain_mesh, const std::filesystem::path &path, const std::unordered_map<std::string, std::string> extra_metadata = {}) {
    // ********************* Preprocessing ********************* //
    // Calculate the average vertex position for later normalization.
    glm::dvec3 average_position(0, 0, 0);
    for (size_t i = 0; i < terrain_mesh.vertex_count(); i++) {
        average_position += terrain_mesh.positions[i] / static_cast<double>(terrain_mesh.vertex_count());
    }

    // Create vertex data vector from positions and uvs.
    // We also normalize vertex position by extracting their average position and storing the offsets.
    // This is dont to preserve our double accuracy, as gltf cannot store them directly.
    std::vector<float> vertices;
    vertices.reserve((vectorsizeof(terrain_mesh.positions) + vectorsizeof(terrain_mesh.uvs)) / sizeof(float));
    glm::vec3 max_position(std::numeric_limits<float>::min());
    glm::vec3 min_position(std::numeric_limits<float>::max());
    for (size_t i = 0; i < terrain_mesh.vertex_count(); i++) {
        const glm::vec3 normalized_position = terrain_mesh.positions[i] - average_position;

        vertices.push_back(normalized_position.x);
        vertices.push_back(normalized_position.y);
        vertices.push_back(normalized_position.z);
        vertices.push_back(terrain_mesh.uvs[i].x);
        vertices.push_back(terrain_mesh.uvs[i].y);

        max_position = glm::max(max_position, normalized_position);
        min_position = glm::min(min_position, normalized_position);
    }

    // Encode the texture as jpeg data.
    const bool has_texture = terrain_mesh.texture.has_value();
    std::vector<uint8_t> texture_bytes;
    if (has_texture) {
        texture_bytes = terrain_mesh.texture->save_to_vector(FIF_JPEG);
    }

    // Create a single buffer that holds all binary data (indices, vertices, textures)
    // We need to do this because only a single buffer can be written as a binary blob in .glb files.
    const size_t index_data_offset = align(sizeof(glm::uvec3), 0);
    const size_t index_data_byte_count = vectorsizeof(terrain_mesh.triangles);
    const size_t index_data_end = index_data_offset + index_data_byte_count;
    const size_t vertex_data_offset = align(sizeof(glm::vec3), index_data_end);
    const size_t vertex_data_byte_count = vectorsizeof(vertices);
    const size_t vertex_data_end = vertex_data_offset + vertex_data_byte_count;
    const size_t texture_data_offset = align(sizeof(glm::uint8_t), vertex_data_end);
    const size_t texture_data_byte_count = vectorsizeof(texture_bytes);
    const size_t texture_data_end = texture_data_offset + texture_data_byte_count;

    std::vector<uint8_t> buffer_data;
    buffer_data.resize(texture_data_end);
    memcpy(buffer_data.data() + index_data_offset, terrain_mesh.triangles.data(), index_data_byte_count);
    memcpy(buffer_data.data() + vertex_data_offset, vertices.data(), vertex_data_byte_count);
    memcpy(buffer_data.data() + texture_data_offset, texture_bytes.data(), texture_data_byte_count);

    const bool binary_output = path.extension() == ".glb";

    // ********************* Create GLTF data structure ********************* //

    // Initialize a GLTF data structure
    cgltf_data data = {};
    std::string version = "2.0\0";
    data.asset.version = version.data();
    std::string generator = "cgltf\0";
    data.asset.generator = generator.data();

    std::vector<cgltf_buffer> buffers;

    // Create a gltf buffer to hold vertex data, index data and the texture.
    cgltf_buffer& buffer = buffers.emplace_back();
    buffer.size = buffer_data.size();
    buffer.data = buffer_data.data();
    std::string buffer_data_encoded; // needs to be declared here to stay alive for long enough
    if (binary_output) {
        // The binary blob at the end of the file will be used as the contents of the first buffer if it does not have an uri defined.
        data.bin = buffer_data.data();
        data.bin_size = buffer_data.size();
    } else {
        buffer_data_encoded = data_uri_encode(buffer_data.data(), buffer_data.size());
        buffer.uri = buffer_data_encoded.data();
    }

    // Create buffer views for each of types of data in the buffer.
    std::vector<cgltf_buffer_view> buffer_views;
    buffer_views.reserve(3);

    cgltf_buffer_view &index_buffer_view = buffer_views.emplace_back();
    index_buffer_view.buffer = &buffer;
    index_buffer_view.offset = index_data_offset;
    index_buffer_view.size = index_data_byte_count;
    index_buffer_view.stride = sizeof(uint32_t);
    index_buffer_view.type = cgltf_buffer_view_type_indices;

    cgltf_buffer_view &vertex_buffer_view = buffer_views.emplace_back();
    vertex_buffer_view.buffer = &buffer;
    vertex_buffer_view.offset = vertex_data_offset;
    vertex_buffer_view.size = vertex_data_byte_count;
    vertex_buffer_view.stride = sizeof(float) * 5;
    vertex_buffer_view.type = cgltf_buffer_view_type_vertices;

    cgltf_buffer_view *texture_buffer_view = nullptr;
    if (has_texture) {
        texture_buffer_view = &buffer_views.emplace_back();
        texture_buffer_view->buffer = &buffer;
        texture_buffer_view->offset = texture_data_offset;
        texture_buffer_view->size = texture_data_byte_count;
        texture_buffer_view->stride = sizeof(uint8_t);
    }

    // Create accessors describing the layout of data in the buffer views.
    std::vector<cgltf_accessor> accessors;
    accessors.reserve(3);

    // Create an accessor for indices.
    cgltf_accessor &index_accessor = accessors.emplace_back();
    index_accessor.buffer_view = &index_buffer_view;
    index_accessor.type = cgltf_type_scalar;
    index_accessor.component_type = cgltf_component_type_r_32u;
    index_accessor.count = terrain_mesh.face_count() * 3;
    index_accessor.offset = 0;
    index_accessor.has_min = true;
    index_accessor.has_max = true;
    index_accessor.min[0] = static_cast<cgltf_float>(0.0);
    index_accessor.max[0] = static_cast<cgltf_float>(terrain_mesh.vertex_count() - 1);

    // Create an accessor for vertex positions
    cgltf_accessor &position_accessor = accessors.emplace_back();
    position_accessor.buffer_view = &vertex_buffer_view;
    position_accessor.component_type = cgltf_component_type_r_32f;
    position_accessor.type = cgltf_type_vec3;
    position_accessor.offset = 0 * sizeof(float);
    position_accessor.stride = 5 * sizeof(float);
    position_accessor.count = terrain_mesh.vertex_count();
    // We need the min and max as some viewers otherwise refuse to open the file.
    position_accessor.has_min = true;
    position_accessor.has_max = true;
    std::copy(glm::value_ptr(min_position), glm::value_ptr(min_position) + min_position.length(), position_accessor.min);
    std::copy(glm::value_ptr(max_position), glm::value_ptr(max_position) + max_position.length(), position_accessor.max);

    // Create an accessor for vertex uvs
    cgltf_accessor &uv_accessor = accessors.emplace_back();
    uv_accessor.buffer_view = &vertex_buffer_view;
    uv_accessor.component_type = cgltf_component_type_r_32f;
    uv_accessor.type = cgltf_type_vec2;
    uv_accessor.offset = 3 * sizeof(float);
    uv_accessor.stride = 5 * sizeof(float);
    uv_accessor.count = terrain_mesh.vertex_count();
    uv_accessor.has_min = true;
    uv_accessor.has_max = true;
    std::fill(uv_accessor.min, uv_accessor.min + 2, 0);
    std::fill(uv_accessor.max, uv_accessor.max + 2, 1);

    // Create a mesh primitive.
    std::vector<cgltf_attribute> primitive_attributes;
    primitive_attributes.reserve(2);
    cgltf_attribute &position_attribute = primitive_attributes.emplace_back();
    std::string position_attribute_name = "POSITION\0";
    position_attribute.name = position_attribute_name.data();
    position_attribute.type = cgltf_attribute_type::cgltf_attribute_type_position;
    position_attribute.index = 0;
    position_attribute.data = &position_accessor;
    cgltf_attribute &uv_attribute = primitive_attributes.emplace_back();
    std::string uv_attribute_name = "TEXCOORD_0\0";
    uv_attribute.name = uv_attribute_name.data();
    uv_attribute.type = cgltf_attribute_type::cgltf_attribute_type_texcoord;
    uv_attribute.index = 0;
    uv_attribute.data = &uv_accessor;

    // Create a gltf texture
    std::vector<cgltf_image> images;
    images.reserve(1);
    cgltf_image &image = images.emplace_back();
    image.buffer_view = texture_buffer_view;
    std::string image_mime_type = "image/jpeg\0";
    image.mime_type = image_mime_type.data();

    std::vector<cgltf_sampler> samplers;
    samplers.reserve(1);
    cgltf_sampler &sampler = samplers.emplace_back();
    sampler.min_filter = GL_LINEAR_MIPMAP_LINEAR;
    sampler.mag_filter = GL_LINEAR;
    sampler.wrap_s = GL_CLAMP_TO_EDGE;
    sampler.wrap_t = GL_CLAMP_TO_EDGE;

    std::vector<cgltf_texture> textures;
    textures.reserve(1);
    cgltf_texture &texture = textures.emplace_back();
    texture.image = &image;
    texture.sampler = &sampler;

    // Create a material
    std::vector<cgltf_material> materials;
    materials.reserve(1);
    cgltf_material &material = materials.emplace_back();
    material.has_pbr_metallic_roughness = true;
    material.pbr_metallic_roughness.base_color_factor[0] = 1.0f;
    material.pbr_metallic_roughness.base_color_factor[1] = 0.9f;
    material.pbr_metallic_roughness.base_color_factor[2] = 0.9f;
    material.pbr_metallic_roughness.base_color_factor[3] = 1.0f;
    material.pbr_metallic_roughness.roughness_factor = 1;
    if (has_texture) {
        material.pbr_metallic_roughness.base_color_texture.texture = &texture;
    }
    material.double_sided = true;

    // Build the primitive for the mesh
    std::vector<cgltf_primitive> primitives;
    primitives.reserve(1);
    cgltf_primitive &primitive = primitives.emplace_back();
    primitive.type = cgltf_primitive_type_triangles;
    primitive.indices = &index_accessor;
    primitive.attributes_count = primitive_attributes.size();
    primitive.attributes = primitive_attributes.data();
    primitive.material = &material;

    // Build the actual mesh
    std::vector<cgltf_mesh> meshes;
    meshes.reserve(1);
    cgltf_mesh &mesh = meshes.emplace_back();
    mesh.primitives_count = primitives.size();
    mesh.primitives = &primitive;

    // Create the node hierachy.
    // We create parent nodes to offset the position by the average calculate above.
    // We need multiple parents to ensure that we dont lose our double precision accurary.
    std::vector<cgltf_node> nodes;
    nodes.reserve(3);
    cgltf_node &mesh_node = nodes.emplace_back();
    mesh_node.has_translation = true;
    mesh_node.mesh = &mesh;

    cgltf_node &parent_node = nodes.emplace_back();
    std::vector<cgltf_node *> parent_node_children = {&mesh_node};
    parent_node.children_count = parent_node_children.size();
    parent_node.children = parent_node_children.data();
    parent_node.has_translation = true;

    cgltf_node &parent_parent_node = nodes.emplace_back();
    std::vector<cgltf_node *> parent_parent_node_children = {&parent_node};
    parent_parent_node.children_count = parent_parent_node_children.size();
    parent_parent_node.children = parent_parent_node_children.data();
    parent_parent_node.has_translation = true;

    const glm::vec3 parent_parent_offset(average_position);
    const glm::dvec3 parent_parent_offset_error = glm::dvec3(parent_parent_offset) - average_position;
    const glm::vec3 parent_offset(-parent_parent_offset_error);
    const glm::dvec3 parent_offset_error = glm::dvec3(parent_offset) + glm::dvec3(parent_parent_offset) - average_position;
    const glm::vec3 mesh_offset(-parent_offset_error);
    std::copy(glm::value_ptr(parent_parent_offset), glm::value_ptr(parent_parent_offset) + parent_parent_offset.length(), parent_node.translation);
    std::copy(glm::value_ptr(parent_offset), glm::value_ptr(parent_offset) + parent_offset.length(), parent_node.translation);
    std::copy(glm::value_ptr(mesh_offset), glm::value_ptr(mesh_offset) + mesh_offset.length(), mesh_node.translation);
    const glm::dvec3 full_error = (glm::dvec3(parent_parent_offset) + glm::dvec3(parent_offset) + glm::dvec3(mesh_offset)) - average_position;
    assert(glm::length(full_error) == 0);

    // Create a scene
    std::vector<cgltf_scene> scenes;
    scenes.reserve(1);
    cgltf_scene &scene = scenes.emplace_back();
    std::vector<cgltf_node *> scene_nodes = {&parent_node};
    scene.nodes_count = scene_nodes.size();
    scene.nodes = scene_nodes.data();

    // Set up data references
    data.meshes_count = meshes.size();
    data.meshes = meshes.data();
    data.nodes_count = nodes.size();
    data.nodes = nodes.data();
    data.scenes_count = scenes.size();
    data.scenes = scenes.data();
    data.buffers_count = buffers.size();
    data.buffers = buffers.data();
    data.buffer_views_count = buffer_views.size();
    data.buffer_views = buffer_views.data();
    data.accessors_count = accessors.size();
    data.accessors = accessors.data();
    if (has_texture) {
        data.textures_count = textures.size();
        data.textures = textures.data();
        data.images_count = images.size();
        data.images = images.data();
        data.samplers_count = samplers.size();
        data.samplers = samplers.data();
    }
    data.materials_count = materials.size();
    data.materials = materials.data();

    if (!has_texture) {
        // TODO:
        data.buffer_views_count = 2;
    }

    // Set up extra metadata
    std::string extras_str;
    if (!extra_metadata.empty()) {
        std::stringstream extras = {};
        extras << "{";
        for (auto const &[key, val] : extra_metadata) {
            extras << "\n";
            extras << "    ";
            extras << "\"" << key << "\"";
            extras << ": ";
            extras << val;
            extras << ",";
        }
        extras.seekp(-1, std::ios_base::end); // remove last ","
        extras << "\n  }";
        extras << "\0";
        extras_str = extras.str();
        data.extras.data = extras_str.data();
    }

    // ********************* Save the GLTF data to a file ********************* //
    std::filesystem::create_directories(path.parent_path());
    cgltf_options options;
    if (binary_output) {
        options.type = cgltf_file_type_glb;
    }
    if (cgltf_write_file(&options, path.c_str(), &data) != cgltf_result_success) {
        throw std::runtime_error("Failed to save GLTF file");
    }
}
*/

// TODO: return errors
tl::expected<void, SaveMeshError> save_mesh_to_path(const std::filesystem::path &path, TerrainMesh &mesh) {
    save_mesh_as_gltf2(mesh, path);
    return {};
}
} // namespace io

#endif
