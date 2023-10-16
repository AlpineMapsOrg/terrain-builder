#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <vector>
#include <numeric>

#include <FreeImage.h>
#include <fmt/core.h>
#include <gdal.h>
#include <gdal_priv.h>
#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "Dataset.h"
#include "DatasetReader.h"
#include "ParallelTiler.h"
#include "TileHeightsGenerator.h"
#include "TopDownTiler.h"
#include "alpine_raster.h"
#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "srs.h"
#include "tntn/gdal_init.h"

#include "mesh_io.h"
#include "terrain_mesh.h"
#include "raw_dataset_reader.h"
#include "fi_image.h"

/// Transforms bounds from one srs to another,
/// in such a way that all points inside the original bounds are guaranteed to also be in the new bounds.
/// But there can be points inside the new bounds that were not present in the original ones.
tile::SrsBounds encompassing_bounding_box_transfer(const OGRSpatialReference &source_srs, const OGRSpatialReference &target_srs, const tile::SrsBounds &source_bounds) {
    if (source_srs.IsSame(&target_srs)) {
        return source_bounds;
    }

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

void print_bounds(const tile::SrsBounds &bounds) {
    fmt::print("{} {} {} {}\n", bounds.min.x, bounds.min.y, bounds.size().x, bounds.size().y);
}
void print_bounds(const geometry::Aabb2i &bounds) {
    fmt::print("{} {} {} {}\n", bounds.min.x, bounds.min.y, bounds.size().x, bounds.size().y);
}
template <typename T>
void print_point(const glm::tvec2<T> &point) {
    fmt::print("{} {}\n", point.x, point.y);
}

template <typename T>
glm::dvec2 apply_transform(std::array<double, 6> transform, const glm::tvec2<T> &v) {
    glm::dvec2 result;
    GDALApplyGeoTransform(transform.data(), v.x, v.y, &result.x, &result.y);
    return result;
}
glm::dvec2 apply_transform(OGRCoordinateTransformation *transform, const glm::dvec2 &v) {
    glm::dvec2 result(v);
    if (!transform->Transform(1, &result.x, &result.y)) {
        throw std::runtime_error("apply_transform() failed");
    }
    return result;
}
glm::dvec3 apply_transform(OGRCoordinateTransformation *transform, const glm::dvec3 &v) {
    glm::dvec3 result(v);
    if (!transform->Transform(1, &result.x, &result.y, &result.z)) {
        throw std::runtime_error("apply_transform() failed");
    }
    return result;
}

void remove_unused_vertices(TerrainMesh& mesh) {
    const unsigned int max_vertex_count = mesh.positions.size();

    // Find used vertices
    std::vector<bool> vertex_present;
    vertex_present.resize(max_vertex_count);
    for (const glm::uvec3 &triangle : mesh.triangles) {
        for (unsigned int i = 0; i < triangle.length(); i++) {
            vertex_present[triangle[i]] = true;
        }
    }

    // Remove unused vertices and remember new indices
    const unsigned int actual_point_count = std::accumulate(vertex_present.begin(), vertex_present.end(), 0);

    if (actual_point_count != max_vertex_count) {
        std::vector<unsigned int> new_vertex_indices;
        new_vertex_indices.reserve(max_vertex_count);

        std::vector<glm::dvec3> old_positions = std::move(mesh.positions);
        std::vector<glm::dvec2> old_uvs = std::move(mesh.uvs);
        mesh.positions.reserve(actual_point_count);
        mesh.uvs.reserve(actual_point_count);
        const unsigned int no_new_index = max_vertex_count + 1;
        unsigned int new_index = 0;
        for (unsigned int i = 0; i < max_vertex_count; i++) {
            if (vertex_present[i]) {
                new_vertex_indices[i] = new_index;
                new_index += 1;
                mesh.positions.push_back(old_positions[i]);
                mesh.uvs.push_back(old_uvs[i]);
            } else {
                new_vertex_indices[i] = no_new_index;
            }
        }

        // Reindex triangles
        for (glm::uvec3 &triangle : mesh.triangles) {
            for (unsigned int i = 0; i < triangle.length(); i++) {
                triangle[i] = new_vertex_indices[triangle[i]];
            }
        }
    }
}

/// Builds a mesh from the given height dataset.
TerrainMesh build_reference_mesh_tile(
    Dataset &dataset,
    const OGRSpatialReference &mesh_srs,
    const OGRSpatialReference &tile_srs, const tile::SrsBounds &tile_bounds,
    const OGRSpatialReference &texture_srs, const tile::SrsBounds &texture_bounds) {
    const OGRSpatialReference &source_srs = dataset.srs();

    // Translate tile bounds from tile srs into the source srs, so we know what data to read.
    const tile::SrsBounds tile_bounds_in_source_srs = encompassing_bounding_box_transfer(tile_srs, source_srs, tile_bounds);

    // Read height data according to bounds directly from dataset (no interpolation).
    RawDatasetReader reader(dataset);
    const geometry::Aabb2i pixel_bounds = reader.transform_srs_bounds_to_pixel_bounds(tile_bounds_in_source_srs);
    const HeightData raw_tile_data = reader.read_data_in_pixel_bounds(pixel_bounds);

    // Allocate mesh data structure
    const unsigned int max_point_count = raw_tile_data.width() * raw_tile_data.height();
    const unsigned int max_triangle_count = (raw_tile_data.width() - 1) * (raw_tile_data.height() - 1) * 2;
    TerrainMesh mesh;
    mesh.positions.reserve(max_point_count);
    mesh.uvs.reserve(max_point_count);
    mesh.triangles.reserve(max_triangle_count);

    // Prepare transformations here to avoid io inside the loop.
    const std::unique_ptr<OGRCoordinateTransformation> transform_source_tile = srs::transformation(source_srs, tile_srs);
    const std::unique_ptr<OGRCoordinateTransformation> transform_source_mesh = srs::transformation(source_srs, mesh_srs);
    const std::unique_ptr<OGRCoordinateTransformation> transform_source_texture = srs::transformation(source_srs, texture_srs);

    for (unsigned int j = 0; j < raw_tile_data.height(); j++) {
        for (unsigned int i = 0; i < raw_tile_data.width(); i++) {
            const double height = raw_tile_data.pixel(i, j);

            const glm::dvec2 coords_raster_relative(i + 0.5, j + 0.5);
            const glm::dvec2 coords_raster_absolute = coords_raster_relative + glm::dvec2(pixel_bounds.min);
            const glm::dvec3 coords_source(reader.transform_pixel_to_srs_point(coords_raster_absolute), height);

            const glm::dvec3 coords_mesh = apply_transform(transform_source_mesh.get(), coords_source);
            // const glm::dvec2 coords_tile = apply_transform(transform_source_tile.get(), glm::dvec2(coords_source));

            const glm::dvec2 coords_texture_absolute = apply_transform(transform_source_texture.get(), coords_source);
            const glm::dvec2 coords_texture_relative = (coords_texture_absolute - texture_bounds.min) / texture_bounds.size();

            const glm::ivec2 coords_center_raster_relative(i + 1, j + 1);
            const glm::ivec2 coords_center_raster_absolute = coords_center_raster_relative + pixel_bounds.min;
            const glm::dvec2 coords_center_source = reader.transform_pixel_to_srs_point(coords_center_raster_absolute);
            const glm::dvec2 coords_center_tile = apply_transform(transform_source_tile.get(), coords_center_source);

            // Current Vertex
            mesh.positions.push_back(coords_mesh);
            mesh.uvs.push_back(coords_texture_relative);

            if (i >= raw_tile_data.width() - 1 || j >= raw_tile_data.width() - 1) {
                // Skip last row and column because they don't form new triangles
                continue;
            }

            if (!tile_bounds.contains_inclusive(coords_center_tile)) {
                // Skip triangles out of bounds
                continue;
            }

            // Add triangles
            const unsigned int vertex_index = i * raw_tile_data.width() + j;
            mesh.triangles.emplace_back(vertex_index, vertex_index + raw_tile_data.width(), vertex_index + raw_tile_data.width() + 1);
            mesh.triangles.emplace_back(vertex_index, vertex_index + raw_tile_data.width() + 1, vertex_index + 1);
        }
    }

    assert(mesh.positions.size() == max_point_count);
    assert(mesh.uvs.size() == max_point_count);
    assert(mesh.triangles.size() <= max_triangle_count);

    remove_unused_vertices(mesh);

    return mesh;
}

// Searches for the smallest webmercator tile that encompasses the given bounding box.
tile::Id bounds_to_tile_id(const tile::SrsBounds &bounds) {
    // We dont want to recurse indefinetely if the bounds are empty.
    if (bounds.width() == 0 || bounds.height() == 0) {
        throw std::invalid_argument("bounds cannot be empty");
    }

    const ctb::Grid grid = ctb::GlobalMercator();
    const std::array<glm::dvec2, 4> points = {
        bounds.min,
        bounds.max,
        glm::dvec2(bounds.min.x, bounds.max.y),
        glm::dvec2(bounds.max.x, bounds.min.y)};

    // We start at the root tile and repeatedly check every subtile until we find one that contains all
    // of the bounding box.
    tile::Id current_largest_encompassing_tile = {0, {0, 0}, tile::Scheme::Tms};
    while (true) {
        const std::array<tile::Id, 4> children = current_largest_encompassing_tile.children();

        bool all_points_inside_any_child = false;
        for (const auto &child : children) {
            // Get the bounds of the current child
            const tile::SrsBounds tile_bounds = grid.srsBounds(child, false);

            // Check if all of the target bounding box is inside this tile.
            bool all_points_inside = true;
            for (const auto &point : points) {
                if (!tile_bounds.contains_inclusive(point)) {
                    all_points_inside = false;
                    break;
                }
            }

            // If all points are inside, we can recurse.
            if (!all_points_inside) {
                continue;
            }

            all_points_inside_any_child = true;
            current_largest_encompassing_tile = child;
            break;
        }

        // Check if all of the points are contained in any children, so we can stop the recursion.
        if (!all_points_inside_any_child) {
            break;
        }
    }

    return current_largest_encompassing_tile;
}

std::vector<unsigned char> prepare_basemap_texture(
    const OGRSpatialReference &target_srs,
    const tile::SrsBounds &target_bounds,
    const std::function<std::filesystem::path(tile::Id)> tile_to_path_mapper) {
    if (target_bounds.width() == 0 || target_bounds.height() == 0) {
        return {};
    }

    OGRSpatialReference webmercator;
    webmercator.importFromEPSG(3857);
    webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    const tile::SrsBounds webmercator_bounds = encompassing_bounding_box_transfer(target_srs, webmercator, target_bounds);
    const tile::Id largest_encompassing_tile = bounds_to_tile_id(webmercator_bounds).to(tile::Scheme::SlippyMap);

    std::vector<tile::Id> tiles_to_splatter;

    std::vector<tile::Id> tile_stack;
    tile_stack.emplace_back(largest_encompassing_tile);

    while (!tile_stack.empty()) {
        const tile::Id tile = tile_stack.back();
        tile_stack.pop_back();

        constexpr size_t subtile_count = 4;
        const std::array<tile::Id, subtile_count> subtiles = tile.children();
        std::array<bool, subtile_count> subtile_present;
        std::array<std::filesystem::path, subtile_count> subtile_path;

        for (size_t i = 0; i < subtile_count; i++) {
            subtile_path[i] = tile_to_path_mapper(subtiles[i]);
            subtile_present[i] = !subtile_path[i].empty() && std::filesystem::exists(subtile_path[i]);
        }

        bool all_present = std::all_of(subtile_present.begin(), subtile_present.end(), [](bool e) { return e; });
        if (!all_present) {
            const std::filesystem::path tile_path = tile_to_path_mapper(tile);
            tiles_to_splatter.push_back(tile);
            std::cout << tile << std::endl;
        }

        for (size_t i = 0; i < subtile_count; i++) {
            if (subtile_present[i]) {
                tile_stack.push_back(subtiles[i]);
            }
        }
    }

    if (tiles_to_splatter.empty()) {
        return {};
    }
    if (tiles_to_splatter.size() == 1 && tiles_to_splatter[0] == largest_encompassing_tile) {
        const std::filesystem::path root_tile_path = tile_to_path_mapper(largest_encompassing_tile);
        if (root_tile_path.empty() || !std::filesystem::exists(root_tile_path)) {
            throw std::runtime_error("found no tile in bounds");
        }
    }

    unsigned int root_zoom_level = largest_encompassing_tile.zoom_level;
    unsigned int min_zoom_level = 0;
    unsigned int max_zoom_level = 0;
    for (const tile::Id &tile : tiles_to_splatter) {
        min_zoom_level = std::min(tile.zoom_level, min_zoom_level);
        max_zoom_level = std::max(tile.zoom_level, max_zoom_level);
    }
    const unsigned int zoom_level_range = max_zoom_level - root_zoom_level;

    const tile::Id &any_tile = tiles_to_splatter.front();
    const std::filesystem::path &any_tile_path = tile_to_path_mapper(any_tile);
    const FiImage any_tile_image = FiImage::load_from_path(any_tile_path);
    const glm::uvec2 tile_size = any_tile_image.size();

    const size_t full_image_size_factor = std::pow(2, zoom_level_range);

    FiImage image = FiImage::allocate_like(any_tile_image, tile_size.x * full_image_size_factor, tile_size.y * full_image_size_factor);

    for (const tile::Id &tile : tiles_to_splatter) {
        const std::filesystem::path &tile_path = tile_to_path_mapper(tile);
        const FiImage tile_image = FiImage::load_from_path(tile_path);
        if (tile_image.width() != tile_size.x) {
            throw std::runtime_error{"tiles have inconsitent widths"};
        }
        if (tile_image.height() != tile_size.y) {
            throw std::runtime_error{"tiles have inconsitent heights"};
        }

        const size_t relative_zoom_level = tile.zoom_level - root_zoom_level;
        const glm::uvec2 current_tile_size_factor = glm::uvec2(std::pow(2, max_zoom_level - tile.zoom_level));
        const glm::uvec2 current_tile_size = tile_size * current_tile_size_factor;
        const glm::uvec2 relative_tile_coords = tile.coords - largest_encompassing_tile.coords * glm::uvec2(std::pow(2, relative_zoom_level));
        const glm::uvec2 current_tile_position = relative_tile_coords * current_tile_size;

        const FiImage scaled_tile_image = tile_image.rescale(current_tile_size, FILTER_BILINEAR);
        image.paste(scaled_tile_image, current_tile_position);
    }

    const std::vector<unsigned char> image_data = image.save_to_vector(FIF_JPEG);

    return image_data;
}

int main() {
    // Read MGI Dataset
    const DatasetPtr dataset = Dataset::make_shared("/mnt/c/Users/Admin/Downloads/innenstadt_gs_1m_mgi.tif");
    // const DatasetPtr dataset = Dataset::make_shared("/mnt/e/Code/TU/2023S/Project/terrain-builder/unittests/data/austria/vienna_20m_mgi.tif");
    // const DatasetPtr dataset = Dataset::make_shared("/mnt/e/Code/TU/2023S/Project/terrain-builder/unittests/data/austria/at_100m_mgi.tif");

    // Convert dataset bounds into google3857
    OGRSpatialReference webmercator;
    webmercator.importFromEPSG(3857);
    webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    OGRSpatialReference wgs84;
    wgs84.importFromEPSG(4326);
    wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    OGRSpatialReference ecef;
    ecef.importFromEPSG(4978);
    ecef.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    // https://mapsneu.wien.gv.at/basemap/bmaporthofoto30cm/normal/google3857/18/90897/142994.jpeg
    const ctb::Grid grid = ctb::GlobalMercator();
    // const tile::Id root_tile = {16, {35748, 22724}, tile::Scheme::SlippyMap};
    // const tile::Id root_tile = {17, {71497, 45448}, tile::Scheme::SlippyMap};
    const tile::Id root_tile = {18, {142994, 90897}, tile::Scheme::SlippyMap};
    // const tile::Id root_tile = {19, {285989, 181795}, tile::Scheme::SlippyMap};.

    // Translate tile bounds into MGI
    const tile::SrsBounds tile_bounds = grid.srsBounds(root_tile, false);

    const TerrainMesh mesh = build_reference_mesh_tile(
        *dataset.get(),
        ecef,
        grid.getSRS(), tile_bounds,
        grid.getSRS(), tile_bounds);

    std::vector<unsigned char> texture_bytes = prepare_basemap_texture(
        grid.getSRS(), tile_bounds,
        [](tile::Id tile_id) { return fmt::format("/mnt/e/Code/TU/2023S/Project/tiles/{}/{}/{}.jpeg", tile_id.zoom_level, tile_id.coords.y, tile_id.coords.x); });

    save_mesh_as_gltf(mesh, texture_bytes);

    return 0;
}
