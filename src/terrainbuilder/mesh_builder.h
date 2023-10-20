#ifndef MESHBUILDING_H
#define MESHBUILDINGE_H

#include <vector>

#include <gdal.h>
#include <gdal_priv.h>
#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "Image.h"
#include "Dataset.h"
#include "srs.h"
#include "tntn/gdal_init.h"

#include "raw_dataset_reader.h"
#include "terrain_mesh.h"

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

template <typename T>
class Border {
public:
    T top;
    T right;
    T bottom;
    T left;

    Border()
        : Border(0) {}
    Border(T uniform)
        : Border(uniform, uniform, uniform, uniform) {}
    Border(T horizontal, T vertical)
        : Border(vertical, horizontal, vertical, horizontal) {}
    Border(T top, T right, T bottom, T left)
        : top(top), right(right), bottom(bottom), left(left) {}

    bool is_empty() const {
        return this->top == 0 && this->right == 0 && this->bottom == 0 && this->left == 0;
    }

    bool operator==(const Border<T> &other) const {
        return top == other.top && right == other.right && bottom == other.bottom && left == other.left;
    }

    bool operator!=(const Border<T> &other) const {
        return !(*this == other);
    }

    friend std::ostream &operator<<(std::ostream &os, const Border<T> &border) {
        os << "Top: " << border.top << ", Right: " << border.right << ", Bottom: " << border.bottom << ", Left: " << border.left;
        return os;
    }
};

template <typename T1, typename T2>
void add_border_to_aabb(geometry::Aabb2<T1> &bounds, const Border<T2> &border) {
    bounds.min.x -= border.left;
    bounds.min.y -= border.top;
    bounds.max.x += border.right;
    bounds.max.y += border.bottom;
}

/// Builds a mesh from the given height dataset.
TerrainMesh build_reference_mesh_tile(
    Dataset &dataset,
    const OGRSpatialReference &mesh_srs,
    const OGRSpatialReference &tile_srs, tile::SrsBounds &tile_bounds,
    const OGRSpatialReference &texture_srs, tile::SrsBounds &texture_bounds,
    const Border<int> &vertex_border,
    const bool inclusive_bounds) {
    const OGRSpatialReference &source_srs = dataset.srs();

    // Translate tile bounds from tile srs into the source srs, so we know what data to read.
    const tile::SrsBounds tile_bounds_in_source_srs = srs::encompassing_bounding_box_transfer(tile_srs, source_srs, tile_bounds);

    // Read height data according to bounds directly from dataset (no interpolation).
    RawDatasetReader reader(dataset);
    geometry::Aabb2i pixel_bounds = reader.transform_srs_bounds_to_pixel_bounds(tile_bounds_in_source_srs);
    add_border_to_aabb(pixel_bounds, vertex_border);
    if (inclusive_bounds) {
        add_border_to_aabb(pixel_bounds, Border(1));
    }
    const HeightData raw_tile_data = reader.read_data_in_pixel_bounds(pixel_bounds);

    // Allocate mesh data structure
    const unsigned int max_vertex_count = raw_tile_data.width() * raw_tile_data.height();
    const unsigned int max_triangle_count = (raw_tile_data.width() - 1) * (raw_tile_data.height() - 1) * 2;
    TerrainMesh mesh;
    mesh.positions.reserve(max_vertex_count);
    mesh.uvs.reserve(max_vertex_count);
    mesh.triangles.reserve(max_triangle_count);

    // Prepare transformations here to avoid io inside the loop.
    const std::unique_ptr<OGRCoordinateTransformation> transform_source_tile = srs::transformation(source_srs, tile_srs);
    const std::unique_ptr<OGRCoordinateTransformation> transform_source_mesh = srs::transformation(source_srs, mesh_srs);
    const std::unique_ptr<OGRCoordinateTransformation> transform_source_texture = srs::transformation(source_srs, texture_srs);

    // Preparation for loop
    const tile::SrsBounds source_bounds_with_border = reader.transform_pixel_bounds_to_srs_bounds(pixel_bounds);
    const double infinity = std::numeric_limits<double>::infinity();
    tile::SrsBounds actual_tile_bounds(glm::dvec2(infinity), glm::dvec2(-infinity));

    std::vector<glm::dvec3> source_positions;
    source_positions.resize(max_vertex_count);

    std::vector<bool> is_in_bounds;
    is_in_bounds.resize(max_vertex_count);
    std::fill(is_in_bounds.begin(), is_in_bounds.end(), false);

    // Pixel values represent the average over their area, or a value at their center.
    const glm::dvec2 point_offset_in_raster(0.5);

    // Iterate over height map and calculate vertex positions for each pixel and whether they are inside the requested bounds.
    for (unsigned int j = 0; j < raw_tile_data.height(); j++) {
        for (unsigned int i = 0; i < raw_tile_data.width(); i++) {
            const unsigned int vertex_index = j * raw_tile_data.width() + i;
            const double height = raw_tile_data.pixel(j, i);

            // Convert pixel coordinates into a point in the dataset's srs.
            const glm::dvec2 coords_raster_relative = glm::dvec2(i, j) + point_offset_in_raster;
            const glm::dvec2 coords_raster_absolute = coords_raster_relative + glm::dvec2(pixel_bounds.min);
            const glm::dvec3 coords_source(reader.transform_pixel_to_srs_point(coords_raster_absolute), height);
            actual_tile_bounds.expand_by(coords_source);
            source_positions[vertex_index] = coords_source;

            // Check if the point is inside the given bounds.
            if (inclusive_bounds) {
                // For inclusive bounds we decide based on the center of the quad,
                // that is spanned from the current vertex to the right and bottom.
                // We do not account for the extra quads on the top and left here,
                // because this is done by adding a 1px border to raw_tile_data above.

                if (i >= raw_tile_data.width() - 1 || j >= raw_tile_data.height() - 1) {
                    // Skip last row and column because they don't form new triangles
                    continue;
                }

                // Transform the center of the quad into the srs the bounds are given.
                const glm::ivec2 coords_center_raster_relative = glm::dvec2(i, j) + point_offset_in_raster + glm::dvec2(0.5);
                const glm::ivec2 coords_center_raster_absolute = coords_center_raster_relative + pixel_bounds.min;
                const glm::dvec2 coords_center_source = reader.transform_pixel_to_srs_point(coords_center_raster_absolute);
                const glm::dvec2 coords_center_tile = apply_transform(transform_source_tile.get(), coords_center_source);

                if (!tile_bounds.contains_inclusive(coords_center_tile)) {
                    continue;
                }
            } else {
                // For exclusive bounds, we only include points (and thereforce triangles) that are (fully) inside the bounds.
                const glm::dvec2 coords_tile = apply_transform(transform_source_tile.get(), coords_source);
                if (!tile_bounds.contains_inclusive(coords_tile)) {
                    continue;
                }
            }

            // If we arrive here the point is inside the bounds.
            is_in_bounds[vertex_index] = true;
        }
    }

    // Mark all border vertices as inside bounds.
    // TODO: This code is rather inefficient for larger borders.
    if (!vertex_border.is_empty()) {
        std::vector<unsigned int> border_vertices;
        const unsigned int max_border_vertices = std::max(raw_tile_data.width(), raw_tile_data.height()); // for each step, assuming convexity
        border_vertices.reserve(max_border_vertices);

        const std::array<std::pair<unsigned int, glm::ivec2>, 4> border_offsets = {
            std::make_pair(vertex_border.left, glm::ivec2(-1, 0)),
            std::make_pair(vertex_border.right, glm::ivec2(1, 0)),
            std::make_pair(vertex_border.top, glm::ivec2(0, -1)),
            std::make_pair(vertex_border.bottom, glm::ivec2(0, 1))};

        for (unsigned int k = 0; k < border_offsets.size(); k++) {
            const unsigned int border_thickness = border_offsets[k].first;
            const glm::ivec2 border_direction = border_offsets[k].second;

            for (unsigned int l = 0; l < border_thickness; l++) {
                for (unsigned int j = 0; j < raw_tile_data.height() - 1; j++) {
                    for (unsigned int i = 0; i < raw_tile_data.width() - 1; i++) {
                        const unsigned int vertex_index = j * raw_tile_data.width() + i;

                        // skip all vertices already inside bounds
                        if (is_in_bounds[vertex_index]) {
                            continue;
                        }

                        // Current vertex index offset into the opposite direction of the current border direction.
                        // So if we are currently making a top border, we offset the current position down.
                        const unsigned int vertex_index_to_check = (j + border_direction.y) * raw_tile_data.width() + (i + border_direction.x);
                        assert(vertex_index_to_check < max_vertex_count);
                        if (is_in_bounds[vertex_index_to_check]) {
                            // as we cannot have a reference into an std::vector<bool> we have to have another lookup here.
                            border_vertices.push_back(vertex_index);
                        }
                    }
                }

                for (const unsigned int border_vertex : border_vertices) {
                    assert(border_vertex < is_in_bounds.size());
                    is_in_bounds[border_vertex] = true;
                }

                assert(border_vertices.size() <= max_border_vertices);
                border_vertices.clear();
            }
        }
    }

    // Compact the vertex array to exclude all vertices out of bounds.
    const unsigned int actual_vertex_count = std::accumulate(is_in_bounds.begin(), is_in_bounds.end(), 0);
    // Marks entries that are not present in the new vector (not inside bounds or border)
    const unsigned int no_new_index = max_vertex_count + 1;
    // Index in old vector mapped to the index in the new one.
    std::vector<unsigned int> new_vertex_indices;
    if (actual_vertex_count != max_vertex_count) {
        new_vertex_indices.reserve(max_vertex_count);

        unsigned int write_index = 0;
        for (unsigned int read_index = 0; read_index < max_vertex_count; read_index++) {
            if (is_in_bounds[read_index]) {
                new_vertex_indices.push_back(write_index);
                source_positions[write_index] = source_positions[read_index];
                write_index += 1;
            } else {
                new_vertex_indices.push_back(no_new_index);
            }
        }

        // Remove out of bounds vertices
        source_positions.resize(actual_vertex_count);
    }

    // Calculate absolute texture coordinates for every vertex.
    tile::SrsBounds actual_texture_bounds(glm::dvec2(infinity), glm::dvec2(-infinity));
    std::vector<glm::dvec2> texture_positions;
    texture_positions.reserve(max_vertex_count);
    for (unsigned int i = 0; i < actual_vertex_count; i++) {
        const glm::dvec3 coords_source = source_positions[i];
        const glm::dvec2 coords_texture_absolute = apply_transform(transform_source_texture.get(), coords_source);
        actual_texture_bounds.expand_by(coords_texture_absolute);
        texture_positions.push_back(coords_texture_absolute);
    }

    // Normalize texture coordinates
    mesh.uvs = std::move(texture_positions);
    for (glm::dvec2 &uv : mesh.uvs) {
        uv = (uv - actual_texture_bounds.min) / actual_texture_bounds.size();
    }
    assert(mesh.uvs.size() == actual_vertex_count);

    // Add triangles
    for (unsigned int j = 0; j < raw_tile_data.height() - 1; j++) {
        for (unsigned int i = 0; i < raw_tile_data.width() - 1; i++) {
            const unsigned int vertex_index = j * raw_tile_data.width() + i;

            std::array<unsigned int, 4> quad = {
                vertex_index,
                vertex_index + 1,
                vertex_index + raw_tile_data.width() + 1,
                vertex_index + raw_tile_data.width(),
            };

            if (inclusive_bounds) {
                // Check if all of the quad is inside the bounds
                bool skip_quad = false;
                for (const unsigned int vertex_index : quad) {
                    if (!is_in_bounds[vertex_index]) {
                        skip_quad = true;
                        break;
                    }
                }
                if (skip_quad) {
                    continue;
                }

                // Calculate the index of the given vertex in the reindexed buffer.
                if (actual_vertex_count != max_vertex_count) {
                    for (unsigned int &vertex_index : quad) {
                        vertex_index = new_vertex_indices[vertex_index];
                        assert(vertex_index != no_new_index);
                    }
                }

                mesh.triangles.emplace_back(quad[0], quad[3], quad[2]);
                mesh.triangles.emplace_back(quad[0], quad[2], quad[1]);
            } else {
                for (unsigned int i = 0; i < 4; i++) {
                    const unsigned int v0 = quad[i];
                    const unsigned int v1 = quad[(i + 1) % 4];
                    const unsigned int v2 = quad[(i + 2) % 4];

                    // Check if the indices are valid
                    if (is_in_bounds[v0] && is_in_bounds[v1] && is_in_bounds[v2]) {
                        mesh.triangles.emplace_back(new_vertex_indices[v0], new_vertex_indices[v1], new_vertex_indices[v2]);
                        i++;
                    }
                }
            }
        }
    }
    assert(mesh.triangles.size() <= max_triangle_count);

    mesh.positions = std::move(source_positions);
    for (glm::dvec3 &position : mesh.positions) {
        position = apply_transform(transform_source_mesh.get(), position);
    }
    assert(mesh.positions.size() == actual_vertex_count);

    tile_bounds = actual_tile_bounds;
    texture_bounds = actual_texture_bounds;

    return mesh;
}

#endif
