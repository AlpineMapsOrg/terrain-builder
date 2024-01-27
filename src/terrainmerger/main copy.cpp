#include <array>
#include <filesystem>
#include <iostream>
#include <optional>
#include <unordered_map>
#include <vector>
#include <span>

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
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
// Midpoint placement policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_placement.h>
// Placement wrapper
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Unique_hash_map.h>
#include <cgltf.h>
#include <fmt/core.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <radix/geometry.h>
#include <CGAL/Surface_mesh_simplification/Edge_collapse_visitor_base.h>

#include "fi_image.h"
#include "gltf_writer.h"
#include "non_copyable.h"

#include "cli.h"
#include "io.h"
#include "merge.h"
#include "simplify.h"
#include "uv_map.h"

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
    cgltf_accessor_unpack_indices(
        &index_accessor,
        reinterpret_cast<unsigned int *>(indices.data()),
        cgltf_component_size(index_accessor.component_type),
        index_accessor.count);

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

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> SurfaceMesh;
typedef CGAL::Surface_mesh_simplification::Edge_profile<SurfaceMesh> Profile;

typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<SurfaceMesh>::edge_descriptor edge_descriptor;
typedef boost::graph_traits<SurfaceMesh>::face_descriptor face_descriptor;

// Stop-condition policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>

// Default cost and placement policies
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_placement.h>

// Non-default cost and placement policies
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_placement.h>

// BGL property map which indicates whether an edge is marked as non-removable
struct Border_is_constrained_edge_map {
    const SurfaceMesh *sm_ptr;
    typedef edge_descriptor key_type;
    typedef bool value_type;
    typedef value_type reference;
    typedef boost::readable_property_map_tag category;
    Border_is_constrained_edge_map(const SurfaceMesh &sm)
        : sm_ptr(&sm) {}
    friend value_type get(const Border_is_constrained_edge_map &m, const key_type &edge) {
        return CGAL::is_border(edge, *m.sm_ptr);
    }
};

// Placement class
typedef CGAL::Surface_mesh_simplification::Constrained_placement<CGAL::Surface_mesh_simplification::Midpoint_placement<SurfaceMesh>,
                                   Border_is_constrained_edge_map> Placement;

typedef SurfaceMesh::Property_map<vertex_descriptor, size_t> SM_pmap;
typedef SurfaceMesh::Property_map<vertex_descriptor, Point_2> UV_pmap;

typedef CGAL::Unique_hash_map<vertex_descriptor, Point_2> UvHashMap;
typedef boost::associative_property_map<UvHashMap> UvPropertyMap;

struct My_visitor : CGAL::Surface_mesh_simplification::Edge_collapse_visitor_base<SurfaceMesh> {
    My_visitor(SM_pmap sm_pmap, UV_pmap uv_pmap)
        : sm_pmap(sm_pmap), uv_pmap(uv_pmap) {}

    // Called during the processing phase for each edge being collapsed.
    // If placement is absent the edge is left uncollapsed.
    void OnCollapsing(const Profile &prof,
                      boost::optional<Point> placement) {
        if (prof.is_v0_v1_a_border() || prof.is_v1_v0_a_border()) {
            assert(false);
        }

        if (placement) {
            vertex_descriptor v0 = prof.v0();
            vertex_descriptor v1 = prof.v1();
            size_t p0_2 = get(sm_pmap, v0);
            size_t p1_2 = get(sm_pmap, v1);
            p_2 = static_cast<double>(p0_2 + p1_2) * 0.5;

            const glm::dvec3 pt = convert::cgal2glm(*placement);
            const glm::dvec3 p0 = convert::cgal2glm(prof.p0());
            const glm::dvec3 p1 = convert::cgal2glm(prof.p1());

            // fmt::println("{}", std::abs(1 - glm::dot(glm::normalize(pt-p0), glm::normalize(p1-p0))));
            // assert(std::abs(1 - glm::dot(glm::normalize(pt-p0), glm::normalize(p1-p0))) < 0.001); // make sure pt is on the edge p0-p1; normalise for numerical stability.
            const auto w1 = std::clamp(glm::length(pt - p0) / glm::length(p1 - p0), 0.0, 1.0);
            // fmt::println("{}", w1);
            // assert(w1 <= 1.0001);
            const auto w0 = 1 - w1;
            
            const glm::dvec2 uv0 = convert::cgal2glm(get(uv_pmap, v0));
            const glm::dvec2 uv1 = convert::cgal2glm(get(uv_pmap, v1));
            this->new_uv = convert::glm2cgal(uv0 * w0 + uv1 * w1);
        }
    }

    // Called after each edge has been collapsed
    void OnCollapsed(const Profile &, vertex_descriptor vd) {
        put(sm_pmap, vd, p_2);
        uv_pmap[vd] = new_uv;
    }

    SM_pmap sm_pmap;
    UV_pmap uv_pmap;
    size_t p_2;
    Point_2 new_uv;
};

std::vector<TerrainMesh> load_meshes_from_path(std::span<const std::filesystem::path> paths, const bool print_errors = true) {
    std::vector<TerrainMesh> meshes;
    meshes.reserve(paths.size());
    for (const auto &path : paths) {
        const auto mesh = io::load_mesh_from_path(path);
        if (!mesh) {
            const io::LoadMeshError error = mesh.error();
            if (print_errors) {
                fmt::println(stderr, "Failed to load mesh from {}: {}", path.string(), error.description());
            }
            return {};
        }
        
        meshes.push_back(std::move(*mesh));
    }

    return meshes;
}

void run(const cli::Args& args) {
    fmt::println("Loading meshes...");
    const std::vector<TerrainMesh> meshes = load_meshes_from_path(args.input_paths);

    fmt::println("Merging meshes...");
    merge::VertexMapping vertex_mapping;
    TerrainMesh merged_mesh = merge::merge_by_distance(meshes, 0.1, vertex_mapping);

    fmt::println("Calculating uv mapping...");
    const uv_map::UvPropertyMap uv_map = uv_map::parameterize_mesh(merged_mesh, uv_map::Algorithm::DiscreteConformalMap, uv_map::Border::Circle);
    merged_mesh.uvs = uv_map::decode_uv_map(uv_map, merged_mesh.vertex_count());

    fmt::println("Merging textures...");
    merged_mesh.texture = convert::cv2fi(uv_map::merge_textures(meshes, merged_mesh, vertex_mapping, uv_map, glm::uvec2(1024)));

    fmt::println("Simplifying merged mesh...");
    const TerrainMesh simplified_mesh = simplify::simplify_mesh(merged_mesh);

    fmt::println("Saving final mesh...");
    io::save_mesh_to_path(args.output_path, simplified_mesh);
}

int main(int argc, char **argv) {
    const cli::Args args = cli::parse(argc, argv);
    run(args);
}

int main2(int argc, char **argv) {
    // Override argc and argv
    argc = 8; // Set the desired number of arguments
    char *new_argv[] = {"./terrainmerger", "--input", "183325_276252.glb", "183325_276253.glb", "183326_276252.glb", "183326_276253.glb", "--output", "./out2.glb"};
    argv = new_argv;

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

    glm::dvec3 average_position(0);
    for (unsigned int i = 0; i < meshes.size(); i++) {
        const auto &mesh = meshes[i];
        for (unsigned int j = 0; j < mesh.positions.size(); j++) {
            const auto &position = mesh.positions[j];
            average_position += position / glm::dvec3(max_combined_vertex_count);
        }
    }

    std::vector<glm::vec3> new_positions;
    std::vector<glm::dvec2> new_uvs;
    std::vector<unsigned int> new_mesh_id;
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
                new_positions.push_back(position - average_position);
                new_uvs.push_back(uv);
                new_mesh_id.push_back(i);
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
    // new_atlas_fi = new_atlas_fi.rescale(glm::uvec2(256));
    new_atlas_fi.save("atlas.png");

    step++;

    cgal_mesh.is_valid(true);

    std::vector<std::vector<std::size_t>> polygons;
    for (const auto& triangle : new_indices) {
        std::vector<std::size_t> polygon;
        for (unsigned int i=0; i<triangle.length(); i++) {
            polygon.push_back(triangle[i]);
        }
        polygons.push_back(polygon);
    }
    assert(CGAL::Polygon_mesh_processing::is_polygon_soup_a_polygon_mesh(polygons));

    if (!CGAL::is_triangle_mesh(cgal_mesh)) {
        std::cerr << "Input geometry is not triangulated." << std::endl;
        return EXIT_FAILURE;
    }

    /*
    double stop_ratio = 1;
    CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<SurfaceMesh> stop(stop_ratio);
    int r = CGAL::Surface_mesh_simplification::edge_collapse(cgal_mesh, stop);
    std::cout << "\nFinished!\n" << r << " edges removed.\n"
            << cgal_mesh.number_of_vertices() << " vertices edges.\n";
    fmt::println("[{}/{}] Saving merged mesh...", steps, steps);
    */
   /*
    double stop_ratio = 1;
    typedef CGAL::Surface_mesh_simplification::LindstromTurk_cost<SurfaceMesh> LT_cost;
    CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<SurfaceMesh> stop(stop_ratio);
    int r = CGAL::Surface_mesh_simplification::edge_collapse(cgal_mesh, stop, CGAL::parameters::get_cost(LT_cost())
                 .get_placement(CGAL::Surface_mesh_simplification::LindstromTurk_placement<SurfaceMesh>()));
    std::cout << "\nFinished!\n" << r << " edges removed.\n"
            << cgal_mesh.number_of_vertices() << " vertices edges.\n";

            */
    double stop_ratio = 0.25;

    std::cout << "start edge collages." << std::endl;
    CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<SurfaceMesh> stop(stop_ratio);

    typedef CGAL::Surface_mesh_simplification::GarlandHeckbert_policies<SurfaceMesh, Kernel> GH_policies;
    typedef GH_policies::Get_cost GH_cost;
    typedef GH_policies::Get_placement GH_placement;
    typedef CGAL::Surface_mesh_simplification::Bounded_normal_change_placement<GH_placement> Bounded_GH_placement;
     

    bool check_mesh = CGAL::is_valid_polygon_mesh(cgal_mesh);
    std::cout << "vaild or in valid: " << check_mesh << std::endl;
    GH_policies gh_policies(cgal_mesh);
    const GH_cost& gh_cost = gh_policies.get_cost();
    const GH_placement& gh_placement = gh_policies.get_placement();
    Bounded_GH_placement placement(gh_placement);

    SM_pmap sm_pmap = cgal_mesh.add_property_map<vertex_descriptor, size_t>("v:sm").first;
    UV_pmap uv_pmap = cgal_mesh.add_property_map<vertex_descriptor, Point_2>("h:uv").first;
    My_visitor vis(sm_pmap, uv_pmap);
    for (unsigned int i = 0; i < final_uvs.size(); i++) {
        uv_pmap[CGAL::SM_Vertex_index(i)] = glm2cgal(final_uvs[i]);
    }
    for (unsigned int i = 0; i < new_mesh_id.size(); i++) {
        sm_pmap[CGAL::SM_Vertex_index(i)] = new_mesh_id[i];
    }

    std::cout << "Input mesh has " << CGAL::num_vertices(cgal_mesh) << " nv "<< CGAL::num_edges(cgal_mesh) << " ne "
        << CGAL::num_faces(cgal_mesh) << " nf" << std::endl;
    /*internal::cgal_enable_sms_trace = true;*/
    // int r = CGAL::Surface_mesh_simplification::edge_collapse(cgal_mesh, stop, CGAL::parameters::get_cost(gh_cost).get_placement(placement).visitor(vis)); 

    // Contract the surface mesh as much as possible
    // CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<SurfaceMesh> stop(stop_ratio);
    Border_is_constrained_edge_map bem(cgal_mesh);
    // This the actual call to the simplification algorithm.
    // The surface mesh and stop conditions are mandatory arguments.
    int r = CGAL::Surface_mesh_simplification::edge_collapse(cgal_mesh, stop, CGAL::parameters::edge_is_constrained_map(bem).get_placement(Placement(bem)).visitor(vis));

    TerrainMesh final_mesh;
    const size_t vertex_count = CGAL::num_vertices(cgal_mesh);
    const size_t face_count = CGAL::num_faces(cgal_mesh);
    // final_mesh.positions.resize(vertex_count);
    final_mesh.positions.reserve(vertex_count);
    final_mesh.uvs.reserve(vertex_count);
    final_mesh.triangles.resize(face_count);

    std::vector<unsigned int> mapmapmap;
    mapmapmap.resize(vertex_count);

    for (const CGAL::SM_Vertex_index vertex_index : cgal_mesh.vertices()) {
        const Point_3 &vertex = cgal_mesh.point(vertex_index);
        mapmapmap[vertex_index] = final_mesh.positions.size();
        // final_mesh.positions[vertex_index] = glm::dvec3(vertex[0], vertex[1], vertex[2]) + average_position;
        final_mesh.positions.push_back(glm::dvec3(vertex[0], vertex[1], vertex[2]) + average_position);
    }
    for (const glm::dvec3 position : final_mesh.positions) {
        assert(position.x != 0);
    }
    for (const CGAL::SM_Face_index face_index : cgal_mesh.faces()) {
        std::array<unsigned int, 3> triangle;
        unsigned int i = 0;
        for (const CGAL::SM_Vertex_index vertex_index : CGAL::vertices_around_face(cgal_mesh.halfedge(face_index), cgal_mesh)) {
            triangle[i] =  mapmapmap[vertex_index];
            i++;
        }
        final_mesh.triangles.emplace_back(triangle[0], triangle[1], triangle[2]);
    }
    for (const CGAL::SM_Vertex_index vertex_index : cgal_mesh.vertices()) {
        // final_mesh.uvs[vertex_index] = glm::dvec2(0, 0);
    }
    for (const CGAL::SM_Vertex_index vertex_index : cgal_mesh.vertices()) {
        const glm::dvec2 &uv = cgal2glm(uv_pmap[vertex_index]);
        final_mesh.uvs.push_back((uv - glm::dvec2(0.5)) * 0.99 + glm::dvec2(0.5));
    }

    std::vector<glm::vec3> colors;
    for (const CGAL::SM_Vertex_index vertex_index : cgal_mesh.vertices()) {
        const size_t sm = sm_pmap[vertex_index];
        colors.emplace_back(static_cast<float>(sm)/4);
    }
    final_mesh.texture = std::move(new_atlas_fi);

    save_mesh_as_gltf2(final_mesh, output_path);

    return 0;
}
