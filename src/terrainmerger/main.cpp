#include <array>
#include <filesystem>
#include <iostream>
#include <optional>
#include <unordered_map>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <cgltf.h>
#include <fmt/core.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <stb_image_write.h>

#include <radix/geometry.h>

#include "fi_image.h"
#include "gltf_writer.h"
#include "non_copyable.h"
#include "xatlas.h"

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

    static std::optional<RawGltfMesh> load_from_path(const std::filesystem::path &path) {
        cgltf_options options = {};
        cgltf_data *data = NULL;
        cgltf_result result = cgltf_parse_file(&options, path.string().c_str(), &data);
        if (result != cgltf_result::cgltf_result_success) {
            return std::nullopt;
        }
        return std::move(RawGltfMesh(data));
    }

private:
    RawGltfMesh(cgltf_data *data)
        : data(data) {}
};

class Mesh : public NonCopyable {
public:
    std::vector<glm::uvec3> indices;
    std::vector<glm::dvec3> positions;
    std::vector<glm::vec2> uvs;
    std::optional<FiImage> texture;

    Mesh(
        std::vector<glm::uvec3> indices,
        std::vector<glm::dvec3> positions,
        std::vector<glm::vec2> uvs,
        FiImage texture)
        : indices(indices), positions(positions), uvs(uvs), texture(std::move(texture)) {}

    // Move constructor
    Mesh(Mesh &&other) noexcept
        : indices(std::move(other.indices)),
          positions(std::move(other.positions)),
          uvs(std::move(other.uvs)),
          texture(std::move(other.texture)) {
    }

    // Move assignment operator
    Mesh &operator=(Mesh &&other) noexcept {
        if (this != &other) { // Avoid self-assignment
            indices = std::move(other.indices);
            positions = std::move(other.positions);
            uvs = std::move(other.uvs);
            texture = std::move(other.texture);
        }
        return *this;
    }
};

cgltf_attribute *find_attribute_with_type(cgltf_attribute *attributes, size_t attribute_count, cgltf_attribute_type type) {
    for (unsigned int i = 0; i < attribute_count; i++) {
        cgltf_attribute *attribute = &attributes[i];
        if (attribute->type == type) {
            return attribute;
        }
    }

    return nullptr;
}

Mesh load_mesh_from_raw(const RawGltfMesh &raw) {
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
    cgltf_buffer_view &index_buffer_view = *index_accessor.buffer_view;
    const unsigned int *index_data = reinterpret_cast<const unsigned int *>(cgltf_buffer_view_data(&index_buffer_view));
    std::vector<glm::uvec3> indices;
    indices.resize(index_accessor.count / 3);
    cgltf_accessor_unpack_indices(&index_accessor, reinterpret_cast<unsigned int *>(indices.data()), indices.size() * 3);

    cgltf_accessor &position_accessor = *position_attr->data;
    cgltf_accessor &uv_accessor = *uv_attr->data;
    assert(position_accessor.buffer_view == uv_accessor.buffer_view);
    cgltf_buffer_view &vertex_buffer_view = *position_accessor.buffer_view;
    const void *vertex_data = cgltf_buffer_view_data(&vertex_buffer_view);
    const size_t position_size = cgltf_num_components(position_accessor.type) * cgltf_component_size(position_accessor.component_type);
    std::vector<glm::vec3> positions;
    positions.resize(position_accessor.count);
    cgltf_accessor_unpack_floats(&position_accessor, reinterpret_cast<float *>(positions.data()), positions.size() * 3);
    const size_t uv_size = cgltf_num_components(uv_accessor.type) * cgltf_component_size(uv_accessor.component_type);
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
    assert(intermediate_node.children_count == 0);
    glm::dvec3 offset =
        glm::dvec3(root_node.translation[0], root_node.translation[1], root_node.translation[2]) +
        glm::dvec3(intermediate_node.translation[0], intermediate_node.translation[1], intermediate_node.translation[2]);

    cgltf_material &material = *mesh_primitive.material;
    assert(material.has_pbr_metallic_roughness);
    assert(material.pbr_metallic_roughness.base_color_texture.texture != nullptr);
    cgltf_texture &albedo_texture = *material.pbr_metallic_roughness.base_color_texture.texture;
    cgltf_image &albedo_image = *albedo_texture.image;
    cgltf_sampler &albedo_sampler = *albedo_texture.sampler;

    FiImage texture = FiImage::load_from_buffer(
        cgltf_buffer_view_data(albedo_image.buffer_view),
        albedo_image.buffer_view->size);

    std::vector<glm::dvec3> positionsd;
    positionsd.resize(positions.size());
    for (unsigned int i = 0; i < positionsd.size(); i++) {
        positionsd[i] = glm::dvec3(positions[i]) + offset;
    }

    return std::move(Mesh(indices, positionsd, uvs, std::move(texture)));
}

template <typename T>
class Octree {
public:
    Octree(glm::vec3 center, float size)
        : center(center), extends(size / 2.0f) {
        children.reserve(8);
    }

    T *find(glm::vec3 point, float epsilon = 0.00001f) {
        if (!this->is_in_bounds(point)) {
            return nullptr;
        }

        if (this->has_value()) {
            if (glm::distance(point, this->point) < epsilon) {
                return &this->value;
            }
        } else {
            for (int i = 0; i < 8; i++) {
                T *value = children[i].find(point, epsilon);
                if (value) {
                    return value;
                }
            }
        }

        return nullptr;
    }

    void insert(glm::vec3 point, T value) {
        if (!this->is_in_bounds(point)) {
            return;
        }

        if (this->has_value()) {
            this->split();

            for (int i = 0; i < 8; i++) {
                children[i].insert(point, value);
            }
        } else {
            this->point = point;
            this->value = std::move(value);
        }
    }

    bool has_value() {
        return !this->has_children();
    }

    bool has_children() {
        return !this->children.empty();
    }
    // private:

    bool is_in_bounds(glm::vec3 point) {
        const glm::vec3 min_bounds = center - glm::vec3(extends);
        const glm::vec3 max_bounds = center + glm::vec3(extends);
        return glm::all(glm::greaterThanEqual(point, min_bounds)) && glm::all(glm::lessThan(point, max_bounds));
    }

    void split() {
        assert(!this->has_children());

        for (int i = 0; i < 8; i++) {
            glm::vec3 child_center = center;
            child_center.x += (i & 1) ? extends : -extends;
            child_center.y += (i & 2) ? extends : -extends;
            child_center.z += (i & 4) ? extends : -extends;
            children.emplace_back(child_center, extends);
        }

        for (int i = 0; i < 8; i++) {
            children[i].insert(std::move(this->point), std::move(this->value));
        }
    }

    glm::vec3 center;
    float extends;
    std::vector<Octree> children;
    glm::vec3 point;
    T value;
};

template <typename T>
class Grid3d {
public:
    Grid3d(glm::vec3 origin, glm::vec3 size, glm::uvec3 divisions)
        : origin(origin), size(size), divisions(divisions) {
        grid_data.resize(divisions.x * divisions.y * divisions.z);
    }

    struct GridCellItem {
        glm::vec3 point;
        T value;
    };

    struct GridCell {
        std::vector<GridCellItem> items;
    };

    T *find(glm::vec3 point, float epsilon = 0.00001f) {
        if (!is_in_bounds(point)) {
            return nullptr;
        }

        int cell_index = this->calculate_cell_index(point);

        GridCell &cell = this->grid_data[cell_index];
        for (GridCellItem &item : cell.items) {
            if (glm::distance(point, item.point) < epsilon) {
                return &item.value;
            }
        }

        return nullptr;
    }

    void insert(glm::vec3 point, T value) {
        if (!is_in_bounds(point)) {
            return;
        }

        int cell_index = this->calculate_cell_index(point);

        GridCellItem item = GridCellItem{
            .point = point,
            .value = value};
        GridCell &cell = grid_data[cell_index];
        cell.items.push_back(item);
    }

    bool is_in_bounds(glm::vec3 point) {
        glm::vec3 maxBounds = origin + size;
        return glm::all(glm::greaterThanEqual(point, origin)) && glm::all(glm::lessThanEqual(point, maxBounds));
    }

    // Calculate the cell index for a given point
    int calculate_cell_index(glm::vec3 point) {
        glm::vec3 cellSize = this->size / glm::vec3(this->divisions);
        glm::uvec3 cellIndex = glm::uvec3((point - this->origin) / cellSize);
        return cellIndex.x + cellIndex.y * divisions.x + cellIndex.z * divisions.x * divisions.y;
    }

    // Other member functions and private members can be added here.

private:
    glm::vec3 origin;
    glm::vec3 size;
    glm::uvec3 divisions;
    std::vector<GridCell> grid_data;
};

struct VertexRef {
    unsigned int mesh;
    unsigned int vertex;
};

static void RandomColor(uint8_t *color) {
    for (int i = 0; i < 3; i++)
        color[i] = uint8_t((rand() % 255 + 192) * 0.5f);
}

static void SetPixel(uint8_t *dest, int destWidth, int x, int y, const uint8_t *color) {
    uint8_t *pixel = &dest[x * 3 + y * (destWidth * 3)];
    pixel[0] = color[0];
    pixel[1] = color[1];
    pixel[2] = color[2];
}

// https://github.com/miloyip/line/blob/master/line_bresenham.c
// License: public domain.
static void RasterizeLine(uint8_t *dest, int destWidth, const int *p1, const int *p2, const uint8_t *color) {
    const int dx = abs(p2[0] - p1[0]), sx = p1[0] < p2[0] ? 1 : -1;
    const int dy = abs(p2[1] - p1[1]), sy = p1[1] < p2[1] ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2;
    int current[2];
    current[0] = p1[0];
    current[1] = p1[1];
    while (SetPixel(dest, destWidth, current[0], current[1], color), current[0] != p2[0] || current[1] != p2[1]) {
        const int e2 = err;
        if (e2 > -dx) {
            err -= dy;
            current[0] += sx;
        }
        if (e2 < dy) {
            err += dx;
            current[1] += sy;
        }
    }
}

/*
https://github.com/ssloy/tinyrenderer/wiki/Lesson-2:-Triangle-rasterization-and-back-face-culling
Copyright Dmitry V. Sokolov

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
static void RasterizeTriangle(uint8_t *dest, int destWidth, const int *t0, const int *t1, const int *t2, const uint8_t *color) {
    if (t0[1] > t1[1])
        std::swap(t0, t1);
    if (t0[1] > t2[1])
        std::swap(t0, t2);
    if (t1[1] > t2[1])
        std::swap(t1, t2);
    int total_height = t2[1] - t0[1];
    for (int i = 0; i < total_height; i++) {
        bool second_half = i > t1[1] - t0[1] || t1[1] == t0[1];
        int segment_height = second_half ? t2[1] - t1[1] : t1[1] - t0[1];
        float alpha = (float)i / total_height;
        float beta = (float)(i - (second_half ? t1[1] - t0[1] : 0)) / segment_height;
        int A[2], B[2];
        for (int j = 0; j < 2; j++) {
            A[j] = int(t0[j] + (t2[j] - t0[j]) * alpha);
            B[j] = int(second_half ? t1[j] + (t2[j] - t1[j]) * beta : t0[j] + (t1[j] - t0[j]) * beta);
        }
        if (A[0] > B[0])
            std::swap(A, B);
        for (int j = A[0]; j <= B[0]; j++)
            SetPixel(dest, destWidth, j, t0[1] + i, color);
    }
}

static void RasterizeTriangle2(std::vector<std::array<uint8_t, 3>> &dest, int destWidth, Mesh &src, std::array<glm::ivec2, 3> dest_uvs, std::array<glm::vec2, 3> source_uvs) {
    glm::ivec2 t0 = dest_uvs[0];
    glm::ivec2 t1 = dest_uvs[1];
    glm::ivec2 t2 = dest_uvs[2];
    glm::vec2 s0 = source_uvs[0];
    glm::vec2 s1 = source_uvs[1];
    glm::vec2 s2 = source_uvs[2];
    if (t0[1] > t1[1]) {
        std::swap(t0, t1);
        std::swap(s0, s1);
    }
    if (t0[1] > t2[1]) {
        std::swap(t0, t2);
        std::swap(s0, s2);
    }
    if (t1[1] > t2[1]) {
        std::swap(t1, t2);
        std::swap(s1, s2);
    }
    int total_height = t2[1] - t0[1];
    for (int i = 0; i < total_height; i++) {
        bool second_half = i > t1[1] - t0[1] || t1[1] == t0[1];
        int segment_height = second_half ? t2[1] - t1[1] : t1[1] - t0[1];
        float alpha = (float)i / total_height;
        float beta = (float)(i - (second_half ? t1[1] - t0[1] : 0)) / segment_height;
        int A[2], B[2];
        for (int j = 0; j < 2; j++) {
            A[j] = int(t0[j] + (t2[j] - t0[j]) * alpha);
            B[j] = int(second_half ? t1[j] + (t2[j] - t1[j]) * beta : t0[j] + (t1[j] - t0[j]) * beta);
        }
        if (A[0] > B[0])
            std::swap(A, B);
        float gamma = 1 - alpha - beta;
        for (int j = A[0]; j <= B[0]; j++) {
            int x = j;
            int y = t0[1] + i;
            glm::ivec2 source_uv = glm::ivec2(alpha * s0 + beta * s1 + gamma * s2);
            RGBQUAD color;
            FreeImage_GetPixelColor(src.texture.value().raw(), source_uv.x, source_uv.y, &color);
            std::array<uint8_t, 3> c = {color.rgbRed, color.rgbGreen, color.rgbBlue};
            SetPixel(dest[0].data(), destWidth, x, y, c.data());
        }
    }
}

cv::Rect clampRectToMatBounds(const cv::Rect &rect, const cv::Mat &mat) {
    int x = std::max(rect.x, 0);
    int y = std::max(rect.y, 0);
    int width = std::max(std::min(rect.width, mat.cols - x), 0);
    int height = std::max(std::min(rect.height, mat.rows - y), 0);

    return cv::Rect(x, y, width, height);
}

void warpTriangle(cv::Mat &img1, cv::Mat &img2, std::array<cv::Point2f, 3> tri1, std::array<cv::Point2f, 3> tri2) {
    // Find bounding rectangle for each triangle
    cv::Rect r1 = clampRectToMatBounds(cv::boundingRect(tri1), img1);
    cv::Rect r2 = clampRectToMatBounds(cv::boundingRect(tri2), img2);
    if (r1.width == 0 || r1.height == 0 || r2.width == 0 || r2.height == 0) {
        return;
    }

    // Offset points by left top corner of the respective rectangles
    std::vector<cv::Point2f> tri1Cropped, tri2Cropped;
    std::vector<cv::Point2i> tri2CroppedInt;
    for (unsigned int i = 0; i < 3; i++) {
        tri1Cropped.push_back(cv::Point2f(tri1[i].x - r1.x, tri1[i].y - r1.y));
        tri2Cropped.push_back(cv::Point2f(tri2[i].x - r2.x, tri2[i].y - r2.y));

        // fillConvexPoly needs a vector of Point and not Point2f
        tri2CroppedInt.push_back(cv::Point2i((int)(tri2[i].x - r2.x), (int)(tri2[i].y - r2.y)));
    }

    // Apply warpImage to small rectangular patches
    cv::Mat img1Cropped;
    img1(r1).copyTo(img1Cropped);

    // Given a pair of triangles, find the affine transform.
    cv::Mat warpMat = cv::getAffineTransform(tri1Cropped, tri2Cropped);

    // Apply the Affine Transform just found to the src image
    cv::Mat img2Cropped = cv::Mat::zeros(r2.height, r2.width, CV_32FC3);
    cv::warpAffine(img1Cropped, img2Cropped, warpMat, img2Cropped.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT_101);

    // Get mask by filling triangle
    cv::Mat mask = cv::Mat::zeros(r2.height, r2.width, CV_32FC3);
    cv::fillConvexPoly(mask, tri2CroppedInt, cv::Scalar(1.0, 1.0, 1.0), 16, 0);

    // Copy triangular region of the rectangular patch to the output image
    cv::multiply(img2Cropped, mask, img2Cropped);
    cv::multiply(img2(r2), cv::Scalar(1.0, 1.0, 1.0) - mask, img2(r2));
    img2(r2) = img2(r2) + img2Cropped;
}

int main() {
    std::vector<std::filesystem::path> paths = {
        "/mnt/e/Code/TU/2023S/Project/meshes/out2/19/181792/285984.glb",
        "/mnt/e/Code/TU/2023S/Project/meshes/out2/19/181792/285985.glb",
        "/mnt/e/Code/TU/2023S/Project/meshes/out/19/181792/285986.glb",
        "/mnt/e/Code/TU/2023S/Project/meshes/out/19/181792/285987.glb"};

    std::vector<Mesh> meshes;
    for (const auto &path : paths) {
        std::optional<RawGltfMesh> raw_mesh = RawGltfMesh::load_from_path(path);
        Mesh mesh = load_mesh_from_raw(std::move(raw_mesh.value()));
        meshes.push_back(std::move(mesh));
    }

    std::vector<unsigned int> index_offsets;
    index_offsets.resize(meshes.size());

    unsigned int current_offset = 0;
    for (unsigned int i = 0; i < meshes.size(); i++) {
        index_offsets[i] = current_offset;
        current_offset += meshes[i].positions.size();
    }

    std::vector<glm::dvec3> new_positions;
    std::vector<glm::dvec2> new_uvs;
    new_positions.reserve(current_offset);
    new_uvs.reserve(current_offset);

    std::vector<glm::dvec2> uv_offsets = {
        glm::dvec2(0, 0),
        glm::dvec2(1, 0),
        glm::dvec2(0, 1),
        glm::dvec2(1, 1)};
    std::vector<glm::dvec2> uv_offsets_abs = {
        glm::dvec2(0, 0),
        glm::dvec2(meshes[0].texture->width(), 0),
        glm::dvec2(0, meshes[0].texture->height()),
        glm::dvec2(meshes[0].texture->width(), meshes[0].texture->height())};
    glm::uvec2 texture_size = meshes[0].texture->size() +
                              glm::uvec2(
                                  std::max(meshes[1].texture->width(), meshes[3].texture->width()),
                                  std::max(meshes[2].texture->height(), meshes[3].texture->height()));

    for (unsigned int i = 0; i < meshes.size(); i++) {
        auto &mesh = meshes[i];

        for (size_t j = 0; j < mesh.positions.size(); j++) {
            new_positions.push_back(mesh.positions[j]);
            new_uvs.push_back((glm::dvec2(mesh.uvs[j]) * glm::dvec2(mesh.texture->size()) + uv_offsets_abs[i]) / glm::dvec2(texture_size));
        }
    }

    std::vector<glm::uvec3> new_indices;
    for (unsigned int i = 0; i < meshes.size(); i++) {
        const auto &mesh = meshes[i];
        for (unsigned int j = 0; j < mesh.indices.size(); j++) {
            const auto &triangle = mesh.indices[j];
            glm::uvec3 new_triangle;
            for (unsigned int k = 0; k < 3; k++) {
                new_triangle[k] = index_offsets[i] + triangle[k];
            }
            new_indices.push_back(new_triangle);
        }
    }

    FiImage new_atlas_fi = FiImage::allocate(meshes[0].texture->type(), texture_size, 24);
    new_atlas_fi.paste(*meshes[0].texture, glm::ivec2(0), true);
    new_atlas_fi.paste(*meshes[1].texture, glm::ivec2(meshes[0].texture->width(), 0), true);
    new_atlas_fi.paste(*meshes[2].texture, glm::ivec2(0, meshes[0].texture->height()), true);
    new_atlas_fi.paste(*meshes[3].texture, glm::ivec2(meshes[0].texture->width(), meshes[0].texture->height()), true);
    new_atlas_fi.save("./output2.png", FIF_PNG);

    TerrainMesh final_mesh;
    final_mesh.triangles = new_indices;
    final_mesh.positions = new_positions;
    final_mesh.uvs = new_uvs;
    final_mesh.texture = std::move(new_atlas_fi);
    save_mesh_as_gltf2(final_mesh, "./out3.glb");

    return 0;
}
