#ifndef GLTFWRITER_H
#define GLTFWRITER_H

#include <filesystem>

#define CGLTF_IMPLEMENTATION
#define CGLTF_WRITE_IMPLEMENTATION
// #define CGLTF_VALIDATE_ENABLE_ASSERTS
#include <cgltf_write.h>
#include <glm/gtc/type_ptr.hpp>

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

using namespace std::literals;

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

std::vector<uint8_t> image_to_bytes(const cv::Mat& image) {
    std::vector<uint8_t> buf;
    cv::imencode(".jpeg", image, buf);
    return buf;
}

/// Saves the mesh as a .gltf or .glb file at the given path.
void save_mesh_as_gltf(const TerrainMesh &m, const std::filesystem::path& path, const std::unordered_map<std::string, std::string> extra_metadata = {}) {
    // ********************* Preprocessing ********************* //

    // Calculate the average vertex position for later normalization.
    const size_t vertex_count = m.positions.size();
    glm::dvec3 average_position(0, 0, 0);
    for (size_t i = 0; i < vertex_count; i++) {
        average_position += m.positions[i] / static_cast<double>(vertex_count);
    }

    // Create vertex data vector from positions and uvs.
    // We also normalize vertex position by extracting their average position and storing the offsets.
    // This is dont to preserve our double accuracy, as gltf cannot store them directly.
    std::vector<float> vertices;
    vertices.reserve((vectorsizeof(m.positions) + vectorsizeof(m.uvs)) / sizeof(float));
    glm::vec3 max_position(std::numeric_limits<float>::min());
    glm::vec3 min_position(std::numeric_limits<float>::max());
    for (size_t i = 0; i < vertex_count; i++) {
        const glm::vec3 normalized_position = m.positions[i] - average_position;

        vertices.push_back(normalized_position.x);
        vertices.push_back(normalized_position.y);
        vertices.push_back(normalized_position.z);
        vertices.push_back(m.uvs[i].x);
        vertices.push_back(m.uvs[i].y);

        max_position = glm::max(max_position, normalized_position);
        min_position = glm::min(min_position, normalized_position);
    }

    // Encode the texture as jpeg data.
    const std::vector<unsigned char> texture_bytes = image_to_bytes(m.texture.value());

    // Create a single buffer that holds all binary data (indices, vertices, textures)
    // We need to do this because only a single buffer can be written as a binary blob in .glb files.
    const size_t index_data_byte_count = vectorsizeof(m.triangles);
    const size_t index_data_offset = 0;
    const size_t index_count = m.triangles.size();
    const size_t vertex_data_byte_count = vectorsizeof(vertices);
    const size_t vertex_data_offset = index_data_offset + index_data_byte_count;
    const size_t texture_data_byte_count = vectorsizeof(texture_bytes);
    const size_t texture_data_offset = vertex_data_offset + vertex_data_byte_count;

    std::vector<unsigned char> buffer_data;
    buffer_data.resize(index_data_byte_count + vertex_data_byte_count + texture_data_byte_count);
    memcpy(buffer_data.data() + index_data_offset, m.triangles.data(), index_data_byte_count);
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
    index_accessor.count = m.triangles.size() * 3;
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
    material.pbr_metallic_roughness.base_color_texture.texture = &texture;
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
    std::array<cgltf_node, 2> nodes;
    cgltf_node &mesh_node = nodes[1] = {};
    mesh_node.has_translation = true;
    mesh_node.mesh = &mesh;

    cgltf_node &parent_node = nodes[0] = {};
    std::array<cgltf_node*, 1> parent_node_children = { &mesh_node };
    parent_node.children_count = parent_node_children.size();
    parent_node.children = parent_node_children.data();
    parent_node.has_translation = true;

    const glm::vec3 parent_offset(average_position);
    const glm::dvec3 parent_offset_error = glm::dvec3(parent_offset) - average_position;
    const glm::vec3 mesh_offset(-parent_offset_error);
    std::copy(glm::value_ptr(parent_offset), glm::value_ptr(parent_offset) + parent_offset.length(), parent_node.translation);
    std::copy(glm::value_ptr(mesh_offset), glm::value_ptr(mesh_offset) + mesh_offset.length(), mesh_node.translation);


    // Create a scene
    std::array<cgltf_scene, 1> scenes;
    cgltf_scene &scene = scenes[0] = {};
    std::array<cgltf_node *, 1> scene_nodes = {&parent_node};
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
    data.textures_count = textures.size();
    data.textures = textures.data();
    data.images_count = images.size();
    data.images = images.data();
    data.samplers_count = samplers.size();
    data.samplers = samplers.data();
    data.materials_count = materials.size();
    data.materials = materials.data();


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
void save_mesh_as_gltf(const TerrainMesh& mesh) {
    // Create a model with a single mesh and save it as a gltf file
    tinygltf::Model m;
    tinygltf::Scene scene;
    tinygltf::Mesh mesh;
    tinygltf::Primitive primitive;
    tinygltf::Node node;
    tinygltf::Buffer buffer;
    tinygltf::BufferView vertex_buffer_view;
    tinygltf::BufferView index_buffer_view;
    tinygltf::Accessor index_accessor;
    tinygltf::Accessor vertex_accessor;
    tinygltf::Asset asset;

    const size_t index_data_byte_count = vectorsizeof(indices);
    const size_t index_data_offset = 0;
    const size_t index_count = indices.size();
    const size_t vertex_data_byte_count = vectorsizeof(vertices);
    const size_t vertex_data_offset = index_data_byte_count;
    const size_t vertex_attributes = 3;
    const size_t vertex_count = vertices.size() / vertex_attributes;

    buffer.data.resize(index_data_byte_count + vertex_data_byte_count);
    memcpy(buffer.data.data() + index_data_offset, indices.data(), index_data_byte_count);
    memcpy(buffer.data.data() + vertex_data_offset, vertices.data(), vertex_data_byte_count);

    index_buffer_view.buffer = 0;
    index_buffer_view.byteOffset = index_data_offset;
    index_buffer_view.byteLength = index_data_byte_count;
    index_buffer_view.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;

    vertex_buffer_view.buffer = 0;
    vertex_buffer_view.byteOffset = vertex_data_offset;
    vertex_buffer_view.byteLength = vertex_data_byte_count;
    vertex_buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

    index_accessor.bufferView = 0;
    index_accessor.byteOffset = 0;
    index_accessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
    index_accessor.count = indices.size();
    index_accessor.type = TINYGLTF_TYPE_SCALAR;
    index_accessor.minValues = {0};
    index_accessor.maxValues = {static_cast<double>(vertices.size() - 1)};

    vertex_accessor.bufferView = 1;
    vertex_accessor.byteOffset = 0;
    vertex_accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    vertex_accessor.count = vertices.size() / vertex_attributes;
    vertex_accessor.type = TINYGLTF_TYPE_VEC3;
    vertex_accessor.minValues = {
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max()
    };
    vertex_accessor.maxValues = {
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::min(),
    };
    for (size_t i = 0; i < vertices.size(); i++) {
        const size_t index = i % vertex_attributes;
        vertex_accessor.minValues[index] = std::min(static_cast<double>(vertices[i]), vertex_accessor.minValues[index]);
        vertex_accessor.maxValues[index] = std::max(static_cast<double>(vertices[i]), vertex_accessor.maxValues[index]);
    }

    // Build the mesh primitive and add it to the mesh
    primitive.indices = 0;                // The index of the accessor for the vertex indices
    primitive.attributes["POSITION"] = 1; // The index of the accessor for positions
    primitive.material = 0;
    primitive.mode = TINYGLTF_MODE_TRIANGLES;
    mesh.primitives.push_back(primitive);

    // Other tie ups
    node.mesh = 0;
    scene.nodes.push_back(0); // Default scene

    // Define the asset. The version is required
    asset.version = "2.0";
    asset.generator = "tinygltf";

    // Now all that remains is to tie back all the loose objects into the
    // our single model.
    m.scenes.push_back(scene);
    m.meshes.push_back(mesh);
    m.nodes.push_back(node);
    m.buffers.push_back(buffer);
    m.bufferViews.push_back(index_buffer_view);
    m.bufferViews.push_back(vertex_buffer_view);
    m.accessors.push_back(index_accessor);
    m.accessors.push_back(vertex_accessor);
    m.asset = asset;

    // Create a simple material
    tinygltf::Material mat;
    mat.pbrMetallicRoughness.baseColorFactor = {1.0f, 0.9f, 0.9f, 1.0f};
    mat.doubleSided = true;
    m.materials.push_back(mat);

    // Save it to a file
    tinygltf::TinyGLTF gltf;
    gltf.WriteGltfSceneToFile(&m, "tile.gltf",
                              true,   // embedImages
                              true,   // embedBuffers
                              true,   // pretty print
                              false); // write binary
}
*/

#endif
