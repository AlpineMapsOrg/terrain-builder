#include <glm/gtc/type_ptr.hpp> 

#include "terrain_mesh.h"
#define CGLTF_IMPLEMENTATION
#define CGLTF_WRITE_IMPLEMENTATION
#include <cgltf_write.h>

using namespace std::literals;

template <typename T>
size_t vectorsizeof(const typename std::vector<T> &vec) {
    return sizeof(T) * vec.size();
}

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

void save_mesh_as_gltf(const TerrainMesh &m) {
    glm::dvec3 average_position(0, 0, 0);
    for (size_t i = 0; i < m.positions.size(); i++) {
        average_position += m.positions[i] / static_cast<double>(m.positions.size());
    }
    std::vector<glm::vec3> positions; positions.resize(m.positions.size());
    for (size_t i = 0; i < m.positions.size(); i++) {
        positions[i] = glm::vec3(m.positions[i] - average_position);
    }

    const size_t index_data_byte_count = vectorsizeof(m.triangles);
    const size_t index_data_offset = 0;
    const size_t index_count = m.triangles.size();
    const size_t vertex_data_byte_count = vectorsizeof(positions);
    const size_t vertex_data_offset = index_data_byte_count;
    const size_t vertex_attributes = 3;
    const size_t vertex_count = positions.size();

    std::vector<unsigned char> buffer_data;
    buffer_data.resize(index_data_byte_count + vertex_data_byte_count);
    memcpy(buffer_data.data() + index_data_offset, m.triangles.data(), index_data_byte_count);
    memcpy(buffer_data.data() + vertex_data_offset, positions.data(), vertex_data_byte_count);

    // Initialize a GLTF data structure
    cgltf_data data = {};
    std::string version = "2.0\0";
    data.asset.version = version.data();
    std::string generator = "cgltf\0";
    data.asset.generator = generator.data();

    // Create a buffer to hold vertex and index data
    std::array<cgltf_buffer, 1> buffers;
    cgltf_buffer &buffer = buffers[0] = cgltf_buffer();
    buffer.size = buffer_data.size();
    buffer.data = buffer_data.data();
    std::string buffer_data_encoded = "data:application/octet-stream;base64," + base64_encode(buffer_data.data(), buffer_data.size());
    buffer.uri = buffer_data_encoded.data();

    std::array<cgltf_buffer_view, 2> buffer_views;

    // Create an accessor for indices
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
    vertex_buffer_view.stride = 0;
    vertex_buffer_view.type = cgltf_buffer_view_type_vertices;

    std::array<cgltf_accessor, 2> accessors;

    // Create an accessor for indices
    cgltf_accessor &index_accessor = accessors[0] = {};
    index_accessor.buffer_view = &index_buffer_view;
    index_accessor.type = cgltf_type_scalar;
    index_accessor.component_type = cgltf_component_type_r_32u;
    index_accessor.count = m.triangles.size() * 3;
    index_accessor.has_min = true;
    index_accessor.min[0] = static_cast<cgltf_float>(0.0);
    index_accessor.has_max = true;
    index_accessor.max[0] = static_cast<cgltf_float>(positions.size() - 1);

    // Create an accessor for vertices
    cgltf_accessor &vertex_accessor = accessors[1] = {};
    vertex_accessor.buffer_view = &vertex_buffer_view;
    vertex_accessor.type = cgltf_type_vec3;
    vertex_accessor.component_type = cgltf_component_type_r_32f;
    vertex_accessor.count = positions.size();
    vertex_accessor.has_min = true;
    vertex_accessor.has_max = true;
    for (size_t i = 0; i < vertex_attributes; i++) {
        vertex_accessor.min[i] = std::numeric_limits<cgltf_float>::max();
        vertex_accessor.max[i] = std::numeric_limits<cgltf_float>::min();
    }
    for (size_t i = 0; i < positions.size(); i++) {
        for (size_t j = 0; j < vertex_attributes; j++) {
            vertex_accessor.min[j] = std::min(vertex_accessor.min[j], static_cast<cgltf_float>(positions[i][j]));
            vertex_accessor.max[j] = std::max(vertex_accessor.max[j], static_cast<cgltf_float>(positions[i][j]));
        }
    }

    std::array<cgltf_attribute, 1> primitive_attributes;
    cgltf_attribute &position_attribute = primitive_attributes[0] = {};
    std::string position_attribute_name = "POSITION\0";
    position_attribute.name = position_attribute_name.data();
    position_attribute.type = cgltf_attribute_type::cgltf_attribute_type_position;
    position_attribute.index = 0;
    position_attribute.data = &vertex_accessor;


    // Create a material
    std::array<cgltf_material, 1> materials;
    cgltf_material &material = materials[0] = {};
    material.has_pbr_metallic_roughness = true;
    material.pbr_metallic_roughness.base_color_factor[0] = 1.0f;
    material.pbr_metallic_roughness.base_color_factor[1] = 0.9f;
    material.pbr_metallic_roughness.base_color_factor[2] = 0.9f;
    material.pbr_metallic_roughness.base_color_factor[3] = 1.0f;
    material.double_sided = true;

    // Build the mesh primitive
    std::array<cgltf_primitive, 1> primitives;
    cgltf_primitive &primitive = primitives[0] = {};
    primitive.type = cgltf_primitive_type_triangles;
    primitive.indices = &index_accessor;
    primitive.attributes_count = primitive_attributes.size();
    primitive.attributes = primitive_attributes.data();
    primitive.material = &material;
    // primitive.has_draco_mesh_compression = true;

    // Create a mesh
    std::array<cgltf_mesh, 1> meshes;
    cgltf_mesh &mesh = meshes[0] = {};
    mesh.primitives_count = 1;
    mesh.primitives = &primitive;

    // Create a node
    std::array<cgltf_node, 1> nodes;
    cgltf_node& node = nodes[0] = {};
    glm::vec3 mesh_offset(average_position);
    node.has_translation = true;
    node.translation[0] = mesh_offset[0];
    node.translation[1] = mesh_offset[1];
    node.translation[2] = mesh_offset[2];
    node.mesh = &mesh;

    // Create a scene
    std::array<cgltf_scene, 1> scenes;
    cgltf_scene &scene = scenes[0] = {};
    std::array<cgltf_node*, 1> scene_nodes = { &node };
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

    if (cgltf_validate(&data) != cgltf_result_success) {
        std::cerr << "Failed to validate mesh data." << std::endl;
    }

    // Save the GLTF data to a file
    const char *output_path = "./tile.gltf";
    cgltf_options options;
    if (cgltf_write_file(&options, output_path, &data) != cgltf_result_success) {
        std::cerr << "Failed to save GLTF file." << std::endl;
    }

    // Free memory
    // cgltf_free(&data);
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