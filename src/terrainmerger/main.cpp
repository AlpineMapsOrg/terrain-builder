#include <array>
#include <filesystem>
#include <iostream>
#include <optional>
#include <unordered_map>
#include <vector>

#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <cgltf.h>
#include <fmt/core.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <radix/geometry.h>

#include "fi_image.h"
#include "gltf_writer.h"
#include "non_copyable.h"

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

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> SurfaceMesh;

typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<SurfaceMesh>::face_descriptor face_descriptor;

int main() {
    std::vector<std::filesystem::path> paths = {
        "/mnt/e/Code/TU/2023S/Project/meshes/out2/19/181792/285984.glb",
        "/mnt/e/Code/TU/2023S/Project/meshes/out2/19/181792/285985.glb",
        "/mnt/e/Code/TU/2023S/Project/meshes/out/19/181792/285986.glb",
        "/mnt/e/Code/TU/2023S/Project/meshes/out/19/181792/285987.glb"
    };

    std::vector<Mesh> meshes;
    for (const auto &path : paths) {
        std::optional<RawGltfMesh> raw_mesh = RawGltfMesh::load_from_path(path);
        Mesh mesh = load_mesh_from_raw(std::move(raw_mesh.value()));
        meshes.push_back(std::move(mesh));
    }

    geometry::Aabb<3, float> bounds;
    for (unsigned int i = 0; i < meshes.size(); i++) {
        const auto &mesh = meshes[i];
        for (unsigned int j = 0; j < mesh.positions.size(); j++) {
            const auto &position = mesh.positions[j];
            bounds.expand_by(position);
        }
    }

    std::vector<std::vector<unsigned int>> index_mapping;
    std::vector<std::unordered_map<unsigned int, unsigned int>> inverse_mapping;

    for (unsigned int i = 0; i < meshes.size(); i++) {
        std::vector<unsigned int> v;
        v.resize(meshes[i].positions.size());
        index_mapping.push_back(std::move(v));

        std::unordered_map<unsigned int, unsigned int> inv;
        inverse_mapping.push_back(inv);
    }

    unsigned int max_combined_vertex_count = 0;
    unsigned int max_combined_index_count = 0;
    for (unsigned int i = 0; i < meshes.size(); i++) {
        max_combined_vertex_count += meshes[i].positions.size();
        max_combined_index_count += meshes[i].indices.size();
    }

    Grid3d<VertexRef> grid(bounds.centre(), bounds.size(), glm::uvec3(100));

    std::vector<glm::dvec3> new_positions;
    std::vector<glm::dvec2> new_uvs;
    new_positions.reserve(max_combined_vertex_count);
    new_uvs.reserve(max_combined_vertex_count);
    for (unsigned int i = 0; i < meshes.size(); i++) {
        auto &mesh = meshes[i];
        for (unsigned int j = 0; j < mesh.positions.size(); j++) {
            auto &position = mesh.positions[j];
            auto &uv = mesh.uvs[j];

            VertexRef *other_vertex = grid.find(position);
            if (other_vertex) {
                // duplicated
                index_mapping[i][j] = index_mapping[other_vertex->mesh][other_vertex->vertex];
                inverse_mapping[i].emplace(index_mapping[i][j], j);
            } else {
                VertexRef vertex_ref{
                    .mesh = i,
                    .vertex = j,
                };
                grid.insert(position, vertex_ref);

                index_mapping[i][j] = new_positions.size();
                inverse_mapping[i].emplace(new_positions.size(), j);
                new_positions.push_back(position);
                new_uvs.push_back(uv);
            }
        }
    }

    std::vector<glm::uvec3> new_indices;
    new_indices.reserve(max_combined_index_count);
    for (unsigned int i = 0; i < meshes.size(); i++) {
        const auto &mesh = meshes[i];
        for (unsigned int j = 0; j < mesh.indices.size(); j++) {
            const auto &triangle = mesh.indices[j];
            glm::uvec3 new_triangle;
            for (unsigned int k = 0; k < 3; k++) {
                new_triangle[k] = index_mapping[i][triangle[k]];
            }
            new_indices.push_back(new_triangle);
        }
    }

    SurfaceMesh cgal_mesh;
    for (const auto &position : new_positions) {
        const auto vertex = cgal_mesh.add_vertex(Point_3(position.x, position.y, position.z));
        assert(vertex != SurfaceMesh::null_vertex());
    }
    for (const auto &triangle : new_indices) {
        const auto face = cgal_mesh.add_face(
            CGAL::SM_Vertex_index(triangle.x),
            CGAL::SM_Vertex_index(triangle.y),
            CGAL::SM_Vertex_index(triangle.z));
        assert(face != SurfaceMesh::null_face());
    }

#if _DEBUG
    assert(cgal_mesh.is_valid(true));
#endif

    halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(cgal_mesh).first;
    // The UV property map that holds the parameterized values
    typedef SurfaceMesh::Property_map<vertex_descriptor, Point_2> UV_pmap;

    // The 2D points of the uv parametrisation will be written into this map
    UV_pmap uv_map = cgal_mesh.add_property_map<vertex_descriptor, Point_2>("h:uv").first;

    CGAL::Surface_mesh_parameterization::parameterize(cgal_mesh, bhd, uv_map);

    std::vector<cv::Mat> textures;
    for (unsigned int i = 0; i < meshes.size(); i++) {
        std::vector<uint8_t> buffer = meshes[i].texture->save_to_vector(FIF_PNG);
        cv::Mat mat = cv::imdecode(cv::Mat(1, buffer.size(), CV_8UC1, buffer.data()), cv::IMREAD_UNCHANGED);
        mat.convertTo(mat, CV_32FC3);
        textures.push_back(mat);
    }

    glm::uvec2 atlas_size(1024);
    cv::Mat new_atlas(atlas_size.y, atlas_size.x, CV_32FC3);

    for (const auto &triangle : new_indices) {
        std::array<cv::Point2f, 3> new_uv_triangle;
        for (unsigned int i = 0; i < triangle.length(); i++) {
            const auto uv = uv_map[CGAL::SM_Vertex_index(triangle[i])];
            new_uv_triangle[i] = cv::Point2f(uv.x() * atlas_size.x, uv.y() * atlas_size.y);
        }

        size_t mesh_count = 0;
        for (uint32_t m = 0; m < meshes.size(); m++) {
            bool all_in_mesh = true;
            for (unsigned int i = 0; i < triangle.length(); i++) {
                auto old_index = inverse_mapping[m].find(triangle[i]);
                if (old_index != inverse_mapping[m].end()) {
                    mesh_count += 1;
                    break;
                }
            }
        }

        size_t mesh_index = meshes.size();
        for (uint32_t m = 0; m < meshes.size(); m++) {
            bool all_in_mesh = true;
            for (unsigned int i = 0; i < triangle.length(); i++) {
                auto old_index = inverse_mapping[m].find(triangle[i]);
                if (old_index == inverse_mapping[m].end()) {
                    all_in_mesh = false;
                    break;
                }

                assert(index_mapping[m][old_index->second] == triangle[i]);
            }

            if (all_in_mesh) {
                mesh_index = m;
                break;
            }

            if (mesh_index == meshes.size()) {
                continue;
            }
        }
        assert(mesh_index < meshes.size());
        const Mesh &mesh = meshes[mesh_index];

        std::array<cv::Point2f, 3> old_uv_triangle;
        for (unsigned int i = 0; i < triangle.length(); i++) {
            const auto uv = new_uvs[triangle[i]];
            const auto uv2 = mesh.uvs[inverse_mapping[mesh_index][triangle[i]]];
            // assert(uv.x == mesh.uvs[inverse_mapping[mesh_index][triangle[i]]].x);
            old_uv_triangle[i] = cv::Point2f(uv2.x * mesh.texture->width(), uv2.y * mesh.texture->height());
        }

        if (mesh_count > 1) {
            int a = 1;
        }

        warpTriangle(textures[mesh_index], new_atlas, old_uv_triangle, new_uv_triangle);
    }

    std::vector<glm::dvec2> final_uvs;
    final_uvs.reserve(new_uvs.size());
    for (const Point_2 &uv : uv_map) {
        final_uvs.emplace_back(uv.x(), uv.y());
    }

    std::vector<uint8_t> buf;
    new_atlas.convertTo(new_atlas, CV_8UC3);
    cv::imencode(".png", new_atlas, buf);
    FiImage new_atlas_fi = FiImage::load_from_buffer(buf);

    TerrainMesh final_mesh;
    final_mesh.triangles = new_indices;
    final_mesh.positions = new_positions;
    final_mesh.uvs = final_uvs;
    final_mesh.texture = std::move(new_atlas_fi);
    save_mesh_as_gltf2(final_mesh, "./out3.glb");

    return 0;
}
