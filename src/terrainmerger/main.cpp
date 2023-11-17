#include <array>
#include <filesystem>
#include <iostream>
#include <optional>
#include <unordered_map>
#include <vector>

#include <CLI/CLI.hpp>

#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
// #include <CGAL/Surface_mesh_parameterization/ARAP_parameterizer_3.h>
// #include <CGAL/Surface_mesh_parameterization/Barycentric_mapping_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Circular_border_parameterizer_3.h>
// #include <CGAL/Surface_mesh_parameterization/Discrete_authalic_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Discrete_conformal_map_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Error_code.h>
// #include <CGAL/Surface_mesh_parameterization/Iterative_authalic_parameterizer_3.h>
// #include <CGAL/Surface_mesh_parameterization/Mean_value_coordinates_parameterizer_3.h>
// #include <CGAL/Surface_mesh_parameterization/Square_border_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <CGAL/Unique_hash_map.h>
#include <cgltf.h>
#include <fmt/core.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <radix/geometry.h>

#include <meshoptimizer.h>

#include "fi_image.h"
#include "gltf_writer.h"
#include "non_copyable.h"

#ifndef DEBUG
#define DEBUG !NDEBUG
#endif

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

struct VertexRef {
    unsigned int mesh;
    unsigned int vertex;
};

template <typename T>
class Grid3d {
public:
    Grid3d(glm::dvec3 origin, glm::dvec3 size, glm::uvec3 divisions)
        : origin(origin), size(size), divisions(divisions) {
        grid_data.resize(divisions.x * divisions.y * divisions.z);
    }

    struct GridCellItem {
        glm::dvec3 point;
        T value;
    };

    struct GridCell {
        std::vector<GridCellItem> items;
    };

    std::optional<T> find(glm::dvec3 point, double epsilon = 0.1f) {
        if (!is_in_bounds(point)) {
            fmt::println("Point is out of bounds {} {} {}", point.x, point.y, point.z);
            return std::nullopt;
        }

        const unsigned int cell_index = this->calculate_cell_index(point);

        GridCell &cell = this->grid_data[cell_index];
        std::optional<T> closest_value;
        double closest_distance = std::numeric_limits<double>::infinity();
        for (GridCellItem &item : cell.items) {
            double distance = glm::distance(point, item.point);
            if (distance < epsilon && distance < closest_distance) {
                closest_value = item.value;
                closest_distance = distance;
            }
        }

        return closest_value;
    }

    void insert(glm::dvec3 point, T value) {
        if (!is_in_bounds(point)) {
            return;
        }

        const unsigned int cell_index = this->calculate_cell_index(point);

        const GridCellItem item{
            .point = point,
            .value = value};
        GridCell &cell = grid_data[cell_index];
        cell.items.push_back(item);
    }

    bool is_in_bounds(glm::dvec3 point) {
        const glm::dvec3 max_bounds = origin + size;
        return glm::all(glm::greaterThanEqual(point, origin)) && glm::all(glm::lessThanEqual(point, max_bounds));
    }

    unsigned int calculate_cell_index(glm::dvec3 point) {
        const glm::dvec3 cell_size = this->size / glm::dvec3(this->divisions);
        const glm::uvec3 cell_index = glm::uvec3((point - this->origin) / cell_size);
        return cell_index.x + cell_index.y * divisions.x + cell_index.z * divisions.x * divisions.y;
    }

private:
    glm::dvec3 origin;
    glm::dvec3 size;
    glm::uvec3 divisions;
    std::vector<GridCell> grid_data;
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

void simplify_mesh(TerrainMesh &mesh, float simplification_factor, float simplification_target_error) {
    // start = std::chrono::high_resolution_clock::now();
    
    fmt::println("  Normalizing positions...");

    const size_t vertex_count = mesh.positions.size();
    glm::dvec3 average_position(0, 0, 0);
    for (size_t i = 0; i < vertex_count; i++) {
        average_position += mesh.positions[i] / static_cast<double>(vertex_count);
    }

    std::vector<float> norm_vertices;
    norm_vertices.reserve((mesh.positions.size() * 3) / sizeof(float));
    for (size_t i = 0; i < vertex_count; i++) {
        const glm::vec3 normalized_position = mesh.positions[i] - average_position;
        norm_vertices.push_back(normalized_position.x);
        norm_vertices.push_back(normalized_position.y);
        norm_vertices.push_back(normalized_position.z);
    }

    // 1.) size_t meshopt_simplify(unsigned int* destination, const unsigned int* indices, size_t index_count, const float* vertex_positions, size_t vertex_count, size_t vertex_positions_stride, size_t target_index_count, float target_error, unsigned int options, float* result_error);

    fmt::println("  Simplifying mesh...");

    size_t target_index_count = size_t(mesh.triangles.size() * 3 * simplification_factor);

    std::vector<unsigned int> simplified_indices(mesh.triangles.size() * 3);
    float result_error;
    size_t simplified_index_count = meshopt_simplify(
        simplified_indices.data(),
        reinterpret_cast<const unsigned int*>(mesh.triangles.data()),
        mesh.triangles.size() * 3,
        norm_vertices.data(),
        norm_vertices.size() / 3,
        sizeof(float) * 3,
        target_index_count,
        simplification_target_error,
        0,
        &result_error);
    // size_t simplified_index_count = meshopt_simplify(
    //     simplified_indices.data(),
    //     reinterpret_cast<const unsigned int*>(mesh.triangles.data()),
    //     mesh.triangles.size() * 3,
    //     reinterpret_cast<const float*>(mesh.positions.data()),
    //     mesh.positions.size(),
    //     sizeof(glm::dvec3),
    //     target_index_count,
    //     target_error,
    //     0,
    //     &result_error);

    simplified_indices.resize(simplified_index_count);
    fmt::println("  Input Statistics:");
    fmt::println("    Index count: {}", mesh.triangles.size() * 3);
    fmt::println("    Target index factor: {}", simplification_factor);
    fmt::println("    Target index count: {}", target_index_count);
    fmt::println("    Target error: {}", simplification_target_error);
    fmt::println("  Output Statistics:");
    fmt::println("    Index count: {}", simplified_index_count);
    fmt::println("    Result error: {}", result_error);
    
    // 2a.) MESHOPTIMIZER_API size_t meshopt_optimizeVertexFetchRemap(unsigned int* destination, const unsigned int* indices, size_t index_count, size_t vertex_count);

    fmt::println("  Optimizing vertex fetch...");

    assert(mesh.positions.size() == mesh.uvs.size());

    std::vector<unsigned int> remap(mesh.positions.size());
    size_t remap_count = meshopt_optimizeVertexFetchRemap(
        remap.data(),
        simplified_indices.data(),
        simplified_indices.size(),
        mesh.positions.size());
    remap.resize(remap_count);

    // 2b.) POS: MESHOPTIMIZER_API void meshopt_remapVertexBuffer(void* destination, const void* vertices, size_t vertex_count, size_t vertex_size, const unsigned int* remap);

    fmt::println("  Remapping vertex buffer...");

    std::vector<glm::dvec3> simplified_positions(remap_count);
    meshopt_remapVertexBuffer(
        simplified_positions.data(),
        mesh.positions.data(),
        mesh.positions.size(),
        sizeof(glm::dvec3),
        remap.data());

    // 2c.) TEX: MESHOPTIMIZER_API void meshopt_remapVertexBuffer(void* destination, const void* vertices, size_t vertex_count, size_t vertex_size, const unsigned int* remap);

    fmt::println("  Remapping tex vertex buffer...");

    std::vector<glm::dvec2> simplified_uvs(remap_count);
    meshopt_remapVertexBuffer(
        simplified_uvs.data(),
        mesh.uvs.data(),
        mesh.uvs.size(),
        sizeof(glm::dvec2),
        remap.data());

    // 2d.) IDX: MESHOPTIMIZER_API void meshopt_remapIndexBuffer(unsigned int* destination, const unsigned int* indices, size_t index_count, const unsigned int* remap);

    fmt::println("  Remapping index buffer...");

    std::vector<unsigned int> remapped_simplified_indices(simplified_indices.size());
    meshopt_remapIndexBuffer(
        remapped_simplified_indices.data(),
        simplified_indices.data(),
        simplified_indices.size(),
        remap.data());

    std::vector<glm::uvec3> simplified_triangles(simplified_indices.size() / 3);
    for (size_t i = 0; i < simplified_triangles.size(); i++) {
        simplified_triangles[i] = glm::uvec3(remapped_simplified_indices[i * 3], remapped_simplified_indices[i * 3 + 1], remapped_simplified_indices[i * 3 + 2]);
    }

    mesh.triangles = simplified_triangles;
    mesh.positions = simplified_positions;
    mesh.uvs = simplified_uvs;
}

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> SurfaceMesh;

typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<SurfaceMesh>::face_descriptor face_descriptor;

int main(int argc, char **argv) {

    CLI::App app{"Terrain Merger"};
    // app.allow_windows_style_options();
    argv = app.ensure_utf8(argv);

    std::vector<std::filesystem::path> input_paths;
    app.add_option("--input", input_paths, "Paths to tiles that should be merged")
        ->required()
        ->expected(-1)
        ->check(CLI::ExistingFile);

    std::filesystem::path output_path;
    app.add_option("--output", output_path, "Path to output the merged tile to")
        ->required();

    bool no_mesh_simplification = false;
    app.add_flag("--no-simplify", no_mesh_simplification, "Disable mesh simplification");

    float simplification_factor = 0.25f;
    app.add_option("--simplify-factor", simplification_factor, "Mesh index simplification factor")
        ->check(CLI::Range(0.0f, 1.0f))
        ->excludes("--no-simplify");

    float simplification_target_error = 0.01f;
    app.add_option("--simplify-error", simplification_target_error, "Mesh simplification target error")
        ->check(CLI::Range(0.0f, 1.0f))
        ->excludes("--no-simplify");

    //TODO: Add option to specify the verbosity level

    CLI11_PARSE(app, argc, argv);

    int steps = 6;
    int step = 1;
    if (no_mesh_simplification) {
        steps = 5;
    }

    fmt::println("[{}/{}] Preparing meshes for merging...", step, steps);

    fmt::println("  Loading meshes...");

    std::vector<Mesh> meshes;
    for (const auto &path : input_paths) {
        std::optional<RawGltfMesh> raw_mesh = RawGltfMesh::load_from_path(path);
        Mesh mesh = load_mesh_from_raw(std::move(raw_mesh.value()));
        meshes.push_back(std::move(mesh));
    }

    fmt::println("  Calculating merged mesh AABB...");

    geometry::Aabb<3, double> bounds;
    bounds.min = glm::dvec3(std::numeric_limits<double>::infinity());
    bounds.max = glm::dvec3(-std::numeric_limits<double>::infinity());
    for (unsigned int i = 0; i < meshes.size(); i++) {
        const auto &mesh = meshes[i];
        for (unsigned int j = 0; j < mesh.positions.size(); j++) {
            const auto &position = mesh.positions[j];
            bounds.expand_by(position);
        }
    }
    glm::dvec3 bounds_padding = bounds.size() * 0.01;
    geometry::Aabb<3, double> padded_bounds(bounds.min - bounds_padding, bounds.max + bounds_padding);

    step++;

    fmt::println("[{}/{}]  Merging meshes...", step, steps);

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

    Grid3d<VertexRef> grid(padded_bounds.min, padded_bounds.size(), glm::uvec3(100));

    std::vector<glm::dvec3> new_positions;
    std::vector<glm::dvec2> new_uvs;
    new_positions.reserve(max_combined_vertex_count);
    new_uvs.reserve(max_combined_vertex_count);
    for (unsigned int i = 0; i < meshes.size(); i++) {
        auto &mesh = meshes[i];
        for (unsigned int j = 0; j < mesh.positions.size(); j++) {
            auto &position = mesh.positions[j];
            auto &uv = mesh.uvs[j];

            std::optional<VertexRef> other_vertex = grid.find(position);
            if (other_vertex.has_value()) {
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
            if (new_triangle[0] == new_triangle[1] ||
                new_triangle[1] == new_triangle[2] || 
                new_triangle[2] == new_triangle[0]) {
                fmt::println("  Skipping illegal triangle...");
                continue;
            }
            new_indices.push_back(new_triangle);
        }
    }

    step++;

    fmt::println("[{}/{}] Creating UV parametrization...", step, steps);

    fmt::println("  Setting up CGAL SurfaceMesh...");

    SurfaceMesh cgal_mesh;
    for (const auto &position : new_positions) {
        const auto vertex = cgal_mesh.add_vertex(Point_3(position.x, position.y, position.z));
        assert(vertex != SurfaceMesh::null_vertex());
    }
    for (const auto &triangle : new_indices) {
        auto face = cgal_mesh.add_face(
            CGAL::SM_Vertex_index(triangle.x),
            CGAL::SM_Vertex_index(triangle.y),
            CGAL::SM_Vertex_index(triangle.z));

            if (face == SurfaceMesh::null_face()) {
                fmt::println("Adding face failed, trying again with different order");
                face = cgal_mesh.add_face(
                    CGAL::SM_Vertex_index(triangle.x),
                    CGAL::SM_Vertex_index(triangle.z),
                    CGAL::SM_Vertex_index(triangle.y));
            }
        assert(face != SurfaceMesh::null_face());
    }

#if DEBUG
    assert(cgal_mesh.is_valid(true));
#endif

    fmt::println("  Calculating UV parametrization...");

    halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(cgal_mesh).first;
    // The UV property map that holds the parameterized values
    typedef CGAL::Unique_hash_map<vertex_descriptor, Point_2> UvHashMap;
    typedef boost::associative_property_map<UvHashMap> UvPropertyMap;
    // typedef SurfaceMesh::Property_map<vertex_descriptor, Point_2> UvPropertyMap;

    // The 2D points of the uv parametrisation will be written into this map
    // UvPropertyMap uv_map = cgal_mesh.add_property_map<vertex_descriptor, Point_2>("h:uv").first;
    UvHashMap uv_uhm;
    UvPropertyMap uv_map(uv_uhm);

    typedef CGAL::Surface_mesh_parameterization::Circular_border_uniform_parameterizer_3<SurfaceMesh> BorderParameterizer;
    // typedef CGAL::Surface_mesh_parameterization::Square_border_uniform_parameterizer_3<SurfaceMesh> BorderParameterizer;
    // typedef CGAL::Surface_mesh_parameterization::Discrete_authalic_parameterizer_3<SurfaceMesh, BorderParameterizer> Parameterizer;
    typedef CGAL::Surface_mesh_parameterization::Discrete_conformal_map_parameterizer_3<SurfaceMesh, BorderParameterizer> Parameterizer;
    // typedef CGAL::Surface_mesh_parameterization::Mean_value_coordinates_parameterizer_3<SurfaceMesh, BorderParameterizer> Parameterizer;
    // typedef CGAL::Surface_mesh_parameterization::Iterative_authalic_parameterizer_3<SurfaceMesh, BorderParameterizer> Parameterizer;
    // CGAL::Surface_mesh_parameterization::Error_code err = CGAL::Surface_mesh_parameterization::parameterize(cgal_mesh, Parameterizer(), bhd, uv_map);
    // GAL::Surface_mesh_parameterization::Error_code err = Parameterizer().parameterize(cgal_mesh, bhd, uv_map, 15);
    // typedef CGAL::Surface_mesh_parameterization::ARAP_parameterizer_3<SurfaceMesh, BorderParameterizer> Parameterizer;
    CGAL::Surface_mesh_parameterization::Error_code err = CGAL::Surface_mesh_parameterization::parameterize(cgal_mesh, Parameterizer(), bhd, uv_map);

    if (err != CGAL::Surface_mesh_parameterization::OK) {
        std::cerr << "Error: " << CGAL::Surface_mesh_parameterization::get_error_message(err) << std::endl;
        return EXIT_FAILURE;
    }

    fmt::println("[4/5] Creating texture atlas...");

    fmt::println("  Loading textures...");

    std::vector<cv::Mat> textures;
    for (unsigned int i = 0; i < meshes.size(); i++) {
        std::vector<uint8_t> buffer = meshes[i].texture->save_to_vector(FIF_PNG);
        cv::Mat mat = cv::imdecode(cv::Mat(1, buffer.size(), CV_8UC1, buffer.data()), cv::IMREAD_UNCHANGED);
        mat.convertTo(mat, CV_32FC3);
        textures.push_back(mat);
    }

    fmt::println("  Populating texture atlas...");

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
    // for (const Point_2 &uv : uv_map) {
    //     final_uvs.emplace_back(uv.x(), uv.y());
    // }
    for (unsigned int i = 0; i < new_uvs.size(); i++) {
        const Point_2 &uv = uv_map[CGAL::SM_Vertex_index(i)];
        final_uvs.emplace_back(uv.x(), uv.y());
    }

    fmt::println("  Saving texture atlas...");

    std::vector<uint8_t> buf;
    new_atlas.convertTo(new_atlas, CV_8UC3);
    cv::imencode(".jpeg", new_atlas, buf);
    FiImage new_atlas_fi = FiImage::load_from_buffer(buf);
    new_atlas_fi.rescale(glm::uvec2(1024));
    new_atlas_fi.save("atlas.png");

    step++;

    TerrainMesh final_mesh;
    final_mesh.triangles = new_indices;
    final_mesh.positions = new_positions;
    final_mesh.uvs = final_uvs;
    final_mesh.texture = std::move(new_atlas_fi);

    if (!no_mesh_simplification) {
        fmt::println("[{}/{}] Simplifying mesh...", step, steps);

        simplify_mesh(final_mesh, simplification_factor, simplification_target_error);
        
        step++;
    }

    fmt::println("[{}/{}] Saving merged mesh...", steps, steps);

    save_mesh_as_gltf2(final_mesh, output_path);

    return 0;
}
