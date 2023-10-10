#include <filesystem>
#include <fstream>
#include <optional>

#include <fmt/core.h>
#include <gdal.h>
#include <gdal_priv.h>
#include <glm/glm.hpp>
#include <radix/geometry.h>

#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE
#define TINYGLTF_NO_STB_IMAGE_WRITE
#include <tiny_gltf.h>

#include "ParallelTiler.h"
#include "TileHeightsGenerator.h"
#include "alpine_raster.h"
#include "ctb/Grid.hpp"

#include "Dataset.h"
#include "DatasetReader.h"
#include "TopDownTiler.h"
#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"
#include "srs.h"

tile::SrsBounds encompassing_bounding_box_transfer(const OGRSpatialReference &source_srs, const OGRSpatialReference &target_srs, const tile::SrsBounds &source_bounds) {
    const std::unique_ptr<OGRCoordinateTransformation> transformation = srs::transformation(source_srs, target_srs);
    tile::SrsBounds target_bounds;
    const int result = transformation->TransformBounds(
        source_bounds.min.x, source_bounds.min.y, source_bounds.max.x, source_bounds.max.y,
        &target_bounds.min.x, &target_bounds.min.y, &target_bounds.max.x, &target_bounds.max.y,
        21);
    if (result != TRUE) {
        throw std::runtime_error("encompassing_bounding_box_transfer");
    }
    return target_bounds;
}

HeightData read_raw(const std::shared_ptr<Dataset> &source_dataset, const tile::SrsBounds &bounds) {
    GDALDataset *raw_dataset = source_dataset->gdalDataset();
    GDALRasterBand *heights_band = raw_dataset->GetRasterBand(1); // non-owning pointer

    std::array<double, 6> geo_transform;
    raw_dataset->GetGeoTransform(geo_transform.data());
    std::array<double, 6> inv_geo_transform;
    const int transform_result = GDALInvGeoTransform(geo_transform.data(), inv_geo_transform.data());
    if (transform_result != TRUE) {
        throw std::runtime_error("couldn't transform bounds to raster");
    }

    geometry::Aabb2d pixel_bounds_double;
    GDALApplyGeoTransform(inv_geo_transform.data(), bounds.min.x, bounds.min.y, &pixel_bounds_double.min.x, &pixel_bounds_double.min.y);
    GDALApplyGeoTransform(inv_geo_transform.data(), bounds.max.x, bounds.max.y, &pixel_bounds_double.max.x, &pixel_bounds_double.max.y);

    pixel_bounds_double = geometry::Aabb2d(glm::dvec2(std::min(pixel_bounds_double.min.x, pixel_bounds_double.max.x), std::min(pixel_bounds_double.min.y, pixel_bounds_double.max.y)),
                                           glm::dvec2(std::max(pixel_bounds_double.min.x, pixel_bounds_double.max.x), std::max(pixel_bounds_double.min.y, pixel_bounds_double.max.y)));

    const geometry::Aabb2i pixel_bounds(
        glm::ivec2(static_cast<int>(std::floor(pixel_bounds_double.min.x)), static_cast<int>(std::floor(pixel_bounds_double.min.y))),
        glm::ivec2(static_cast<int>(std::ceil(pixel_bounds_double.max.x)), static_cast<int>(std::ceil(pixel_bounds_double.max.y))));

    // TODO: maybe width and heigth +1
    HeightData heights_data(pixel_bounds.width(), pixel_bounds.height());
    const int read_result = heights_band->RasterIO(GF_Read, pixel_bounds.min.x, pixel_bounds.min.y, pixel_bounds.width(), pixel_bounds.height(),
                                                   static_cast<void *>(heights_data.data()), pixel_bounds.width(), pixel_bounds.height(), GDT_Float32, 0, 0);
    if (read_result != CE_None) {
        throw Exception("couldn't read data");
    }

    return heights_data;
}

class Tiler2 {
public:
    Tiler2(const tile::Id root_tile) {
        this->tile_stack.push_back(root_tile);
    }

    std::optional<tile::Id> next() {
        if (tile_stack.size() == 0) {
            return std::nullopt;
        }

        const tile::Id current_tile = tile_stack.back();
        tile_stack.pop_back();

        const std::array<tile::Id, 4> children = current_tile.children();
        tile_stack.insert(tile_stack.end(), children.begin(), children.end());

        return std::make_optional(current_tile);
    }

private:
    std::vector<tile::Id> tile_stack;
};

void save_mesh(const std::vector<float> vertices, const std::vector<unsigned int> &indices) {
    // Create a model with a single mesh and save it as a gltf file
    tinygltf::Model m;
    tinygltf::Scene scene;
    tinygltf::Mesh mesh;
    tinygltf::Primitive primitive;
    tinygltf::Node node;
    tinygltf::Buffer vertex_buffer;
    tinygltf::Buffer index_buffer;
    tinygltf::BufferView vertex_buffer_view;
    tinygltf::BufferView index_buffer_view;
    tinygltf::Accessor index_accessor;
    tinygltf::Accessor vertex_accessor;
    tinygltf::Asset asset;

    index_buffer.data.resize(indices.size() * sizeof(unsigned int));
    memcpy(index_buffer.data.data(), indices.data(), index_buffer.data.size());
    index_buffer_view.buffer = 0;
    index_buffer_view.byteOffset = 0;
    index_buffer_view.byteLength = index_buffer.data.size();
    index_buffer_view.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;

    vertex_buffer.data.resize(vertices.size() * sizeof(float));
    memcpy(vertex_buffer.data.data(), vertices.data(), vertex_buffer.data.size());
    vertex_buffer_view.buffer = 1;
    vertex_buffer_view.byteOffset = 0;
    vertex_buffer_view.byteLength = vertex_buffer.data.size();
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
    vertex_accessor.count = vertices.size() / 3;
    vertex_accessor.type = TINYGLTF_TYPE_VEC3;
    vertex_accessor.minValues = {
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max()};
    vertex_accessor.maxValues = {
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::min(),
    };
    for (size_t i = 0; i < vertices.size(); i++) {
        const size_t index = i % 3;
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
    m.buffers.push_back(index_buffer);
    m.buffers.push_back(vertex_buffer);
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

void print_bounds(const tile::SrsBounds &bounds) {
    fmt::print("{} {} {} {}\n", bounds.min.x, bounds.min.y, bounds.size().x, bounds.size().y);
}
void print_bounds(const geometry::Aabb2i &bounds) {
    fmt::print("{} {} {} {}\n", bounds.min.x, bounds.min.y, bounds.size().x, bounds.size().y);
}
void print_point(const glm::dvec2 &point) {
    fmt::print("{} {}\n", point.x, point.y);
}
void print_point(const glm::ivec2 &point) {
    fmt::print("{} {}\n", point.x, point.y);
}
glm::dvec2 apply_transform(std::array<double, 6> transform, const glm::dvec2 &v) {
    glm::dvec2 result;
    GDALApplyGeoTransform(transform.data(), v.x, v.y, &result.x, &result.y);
    return result;
}
glm::dvec2 apply_transform(std::array<double, 6> transform, const glm::ivec2 &v) {
    glm::dvec2 result;
    GDALApplyGeoTransform(transform.data(), static_cast<double>(v.x), static_cast<double>(v.y), &result.x, &result.y);
    return result;
}

int main() {
    // Read MGI Dataset
    const DatasetPtr raw_dataset = Dataset::make_shared("/mnt/c/Users/Admin/Downloads/innenstadt_gs_1m_mgi.tif");
    const OGRSpatialReference source_srs = raw_dataset->srs();

    // Convert dataset bounds into google3857
    OGRSpatialReference webmercator;
    webmercator.importFromEPSG(3857);
    webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    OGRSpatialReference wgs84;
    wgs84.importFromEPSG(4326);
    wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    const tile::Id root_tile = {19, {285989, 181795}, tile::Scheme::SlippyMap};

    // Tile regions at specific zoom level
    const unsigned int zoom_level = 21;
    Tiler2 tiler(root_tile);
    const ctb::Grid grid = ctb::GlobalMercator();

    // For Each Tile:
    const tile::Id current_tile = root_tile; // tiler.next().value();
    // Translate tile bounds into MGI
    const tile::SrsBounds tile_bounds = grid.srsBounds(current_tile, false);
    const tile::SrsBounds target_bounds = encompassing_bounding_box_transfer(grid.getSRS(), source_srs, tile_bounds);
    // const tile::SrsBounds target_bounds2 = srs::nonExactBoundsTransform(tile_bounds, grid.getSRS(), source_srs);

    // Read height data according to bounds directly from dataset
    const HeightData raw_tile_data = read_raw(raw_dataset, target_bounds);
    image::debugOut(raw_tile_data, "debug.png");

    // Translate MGI data points into ECEF space
    GDALDataset *gdal_dataset = raw_dataset->gdalDataset();
    GDALRasterBand *heights_band = gdal_dataset->GetRasterBand(1); // non-owning pointer

    std::array<double, 6> geo_transform;
    gdal_dataset->GetGeoTransform(geo_transform.data());
    std::array<double, 6> inv_geo_transform;
    const int transform_result = GDALInvGeoTransform(geo_transform.data(), inv_geo_transform.data());
    if (transform_result != TRUE) {
        throw std::runtime_error("couldn't transform bounds to raster");
    }

    geometry::Aabb2d pixel_bounds_double;
    GDALApplyGeoTransform(inv_geo_transform.data(), target_bounds.min.x, target_bounds.min.y, &pixel_bounds_double.min.x, &pixel_bounds_double.min.y);
    GDALApplyGeoTransform(inv_geo_transform.data(), target_bounds.max.x, target_bounds.max.y, &pixel_bounds_double.max.x, &pixel_bounds_double.max.y);

    pixel_bounds_double = geometry::Aabb2d(glm::dvec2(std::min(pixel_bounds_double.min.x, pixel_bounds_double.max.x), std::min(pixel_bounds_double.min.y, pixel_bounds_double.max.y)),
                                           glm::dvec2(std::max(pixel_bounds_double.min.x, pixel_bounds_double.max.x), std::max(pixel_bounds_double.min.y, pixel_bounds_double.max.y)));

    const geometry::Aabb2i pixel_bounds(
        glm::ivec2(static_cast<int>(std::floor(pixel_bounds_double.min.x)), static_cast<int>(std::floor(pixel_bounds_double.min.y))),
        glm::ivec2(static_cast<int>(std::ceil(pixel_bounds_double.max.x)), static_cast<int>(std::ceil(pixel_bounds_double.max.y))));

    const unsigned int point_count = raw_tile_data.width() * raw_tile_data.height();
    const unsigned int triangle_count = (raw_tile_data.width() - 1) * (raw_tile_data.height() - 1) * 2;
    std::vector<float> vertices;
    vertices.reserve(point_count * 3);
    std::vector<unsigned int> indices;
    indices.reserve(triangle_count * 3);
    for (unsigned int j = 0; j < raw_tile_data.height(); j++) {
        for (unsigned int i = 0; i < raw_tile_data.width(); i++) {
            unsigned int index = i * raw_tile_data.width() + j;

            const double height = raw_tile_data.pixel(i, j);
            const glm::dvec2 coords_raster(i + 0.5, j + 0.5);
            const glm::dvec3 coords_source(apply_transform(geo_transform, coords_raster), height);
            const glm::vec3 coords_ecef = srs::toECEF(source_srs, coords_source);

            const glm::ivec2 coords_center_raster_local(i + 1, j + 1);
            const glm::ivec2 coords_center_raster_global = coords_center_raster_local + pixel_bounds.min;
            const glm::dvec2 coords_center_source = apply_transform(geo_transform, coords_center_raster_global);
            const glm::dvec2 coords_center_webmercator = srs::to(source_srs, grid.getSRS(), coords_center_source);

            // Current Vertex
            vertices.push_back(coords_ecef.x);
            vertices.push_back(coords_ecef.y);
            vertices.push_back(coords_ecef.z);

            if (i >= raw_tile_data.width() - 1 || j >= raw_tile_data.width() - 1) {
                // Skip last row and column because they don't form new triangles
                continue;
            }

            if (!tile_bounds.contains(coords_center_webmercator)) {
                // Skip triangles out of bounds
                continue;
            }

            // First triangle indices
            indices.push_back(index);
            indices.push_back(index + raw_tile_data.width());
            indices.push_back(index + raw_tile_data.width() + 1);
            // Second triangle
            indices.push_back(index);
            indices.push_back(index + raw_tile_data.width() + 1);
            indices.push_back(index + 1);
        }
    }

    save_mesh(vertices, indices);

    return 0;
}
