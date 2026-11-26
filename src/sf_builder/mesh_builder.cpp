/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2021 Adam Celarek-Litofcenko
 * Copyright (C) 2021 Martin Braunsperger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <optional>
#include <ranges>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <radix/geometry.h>
#include <radix/raster.h>
#include <glm/gtx/norm.hpp>
#include <libassert/assert.hpp>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include "cli_support/log.h"
#include "error/try.h"
#include "error/utils.h"
#include "fmt_impls.h"
#include "geometry/geometry.h"
#include "gis/Dataset.h"
#include "gis/srs/Kind.h"
#include "gis/srs/System.h"
#include "gis/srs/Transform.h"
#include "gis/srs/coverage.h"
#include "gis/srs/well_known.h"
#include "mesh/SimpleMesh.h"
#include "mesh/cleanup.h"
#include "mesh_builder.h"
#include "native_regions.h"
#include "range_utils.h"
#include "RawDatasetReader.h"
#include "mesh/clip.h"
#include "mesh/combine.h"
#include "mesh/validate/debug_assert.h"
#include "mesh/bounds.h"
#include "texture_assembler.h"
#include "TileProvider.h"

namespace sf {

namespace {
using PixelBounds = radix::geometry::Aabb2i;
    
glm::dvec3 convert_pixel_to_vertex(const float height, const glm::uvec2 pixel_coords, const RawDatasetReader& reader, const PixelBounds& pixel_bounds) {
    const glm::dvec2 point_offset_in_raster(0.5); // Convert pixel coordinates into a point in the dataset's srs.
    const glm::dvec2 coords_raster_relative = glm::dvec2(pixel_coords) + point_offset_in_raster;
    const glm::dvec2 coords_raster_absolute = coords_raster_relative + glm::dvec2(pixel_bounds.min);
    const glm::dvec3 coords_source(reader.transform_pixel_to_srs_point(coords_raster_absolute), height);
    return coords_source;
}

SimpleMesh meshify(const radix::Raster<glm::dvec3>& source_points, const radix::RasterMask& mask) {
    // Compact the vertex grid into a list of valid ones.
    const size_t valid_vertex_count = sum(mask, size_t{0});
    // Check if we even have any valid vertices. Can happen if all of the region is padding.
    if (valid_vertex_count == 0) {
        return SimpleMesh();
    }

    std::vector<glm::dvec3> positions;
    positions.reserve(valid_vertex_count);

    auto vertex_index_map_result = radix::raster::transform(source_points, mask, [&](const glm::dvec3& point) -> size_t {
        const size_t index = positions.size();
        positions.push_back(point);
        return index;
    });
    const auto vertex_index_map = *ASSERT_VAL(std::move(vertex_index_map_result), "the mask has the size of the source points");
    DEBUG_ASSERT(positions.size() == valid_vertex_count);

    // Allocate triangle vector
    const size_t max_triangle_count = (source_points.width() - 1) * (source_points.height() - 1) * 2;
    std::vector<glm::uvec3> triangles;
    triangles.reserve(max_triangle_count);

    for (unsigned y = 0; y < source_points.height() - 1; y++) {
        for (unsigned x = 0; x < source_points.width() - 1; x++) {
            const std::array<glm::uvec2, 4> quad {
                glm::uvec2{x, y},
                glm::uvec2{x + 1, y},
                glm::uvec2{x + 1, y + 1},
                glm::uvec2{x, y + 1}};

            for (uint32_t i = 0; i < 4; i++) {
                const auto& v0 = quad[i];
                const auto& v1 = quad[(i + 1) % 4];
                const auto& v2 = quad[(i + 2) % 4];

                // Check if the indices are valid
                if (mask.pixel(v0) && mask.pixel(v1) && mask.pixel(v2)) {
                    triangles.emplace_back(vertex_index_map.pixel(v0), vertex_index_map.pixel(v1), vertex_index_map.pixel(v2));
                    i++;
                }
            }
        }
    }
    DEBUG_ASSERT(triangles.size() <= max_triangle_count);

    return SimpleMesh(triangles, positions);
}

alp::Result<SimpleMesh, UntransformableMesh> transform_mesh(SimpleMesh&& source_mesh, const gis::srs::System& source_srs, const gis::srs::System& target_srs) {
    const auto context = alp::make_context("transforming the mesh from {} into {}", source_srs, target_srs);

    auto transform_result = gis::srs::get_transform(source_srs, target_srs);
    ALP_TRY_CONTEXT(const gis::srs::Transform transform, alp::wrap_error(std::move(transform_result), UntransformableMesh{}), context);

    auto apply_result = transform.apply_inplace(&source_mesh.positions);
    ALP_TRY_CONTEXT(alp::wrap_error(std::move(apply_result), UntransformableMesh{}), context);

    return std::move(source_mesh);
}
/*
alp::Result<SimpleMesh, UntransformableMesh> transform_mesh(const SimpleMesh& source_mesh, const gis::srs::System& source_srs, const gis::srs::System& target_srs) {
    SimpleMesh target_mesh;
    auto positions_result = gis::srs::transform_points(source_srs, target_srs, source_mesh.positions);
    ALP_TRY(target_mesh.positions, alp::wrap_error(std::move(positions_result), UntransformableMesh{}));

    target_mesh.triangles = source_mesh.triangles;
    target_mesh.uvs = source_mesh.uvs;
    target_mesh.texture = source_mesh.texture;
    return target_mesh;
}
*/

alp::Result<std::vector<glm::dvec2>, UntransformableMesh> generate_uv_space(const std::vector<glm::dvec3>& positions, const gis::srs::System& mesh_srs, const gis::srs::System& texture_srs, radix::tile::SrsBounds& texture_bounds) {
    const auto context = alp::make_context("generating the uv space in {}", texture_srs);

    auto transform_result = gis::srs::get_transform(mesh_srs, texture_srs);
    ALP_TRY_CONTEXT(const gis::srs::Transform transform, alp::wrap_error(std::move(transform_result), UntransformableMesh{}), context);

    std::vector<glm::dvec2> uvs(positions.size());
    auto apply_result = transform.apply_into(positions, &uvs);
    ALP_TRY_CONTEXT(alp::wrap_error(std::move(apply_result), UntransformableMesh{}), context);

    texture_bounds = radix::tile::SrsBounds(radix::geometry::find_bounds(std::span<const glm::dvec2>(uvs)));

    for (glm::dvec2 &uv : uvs) {
        uv = (uv - texture_bounds.min) / texture_bounds.size();
    }

    return uvs;
}

// The ECEF bounds covering a clip, from which the parts of a dataset to read are computed.
// Supports finite ECEF bounds, used as they are, and geographic or projected bounds, whose heights may be unbounded.
// Fails with UnsupportedClip for other clips, and if the clip bounds cannot be expressed in ECEF.
alp::Result<radix::geometry::Aabb3d, UnsupportedClip> compute_ecef_clip_bounds(const gis::srs::System& clip_srs, const radix::geometry::Aabb3d& clip_bounds) {
    if (clip_srs.is_same(gis::srs::ecef())) {
        if (!geometry::is_finite(clip_bounds)) {
            return alp::make_error(UnsupportedClip{}, "the ECEF clip bounds are not finite");
        }
        return clip_bounds;
    }

    const gis::srs::Kind clip_kind = clip_srs.kind();
    const bool is_geographic_or_projected_clip = clip_kind == gis::srs::Kind::Geographic || clip_kind == gis::srs::Kind::Projected;
    if (!is_geographic_or_projected_clip) {
        return alp::make_error(UnsupportedClip{}, "the clip srs is neither ECEF nor geographic or projected");
    }

    // The source srs is 2D, so the windows for bounds unbounded in height don't depend on the
    // height they are computed at.
    radix::geometry::Aabb3d finite_clip_bounds = clip_bounds;
    if (!std::isfinite(finite_clip_bounds.min.z)) {
        finite_clip_bounds.min.z = std::isfinite(clip_bounds.max.z) ? clip_bounds.max.z : 0;
    }
    if (!std::isfinite(finite_clip_bounds.max.z)) {
        finite_clip_bounds.max.z = finite_clip_bounds.min.z;
    }
    auto ecef_result = gis::srs::coverage::compute_ecef(clip_srs, finite_clip_bounds);
    ALP_TRY(const radix::geometry::Aabb3d ecef_clip_bounds, alp::wrap_error(std::move(ecef_result), UnsupportedClip{}));
    return ecef_clip_bounds;
}

// Meshes the valid heights of the dataset within `window`.
alp::Result<SimpleMesh, UnreadableHeights> read_heights_as_mesh(RawDatasetReader& reader, const radix::tile::SrsBounds& window) {
    radix::geometry::Aabb2i pixel_bounds = reader.transform_srs_bounds_to_pixel_bounds(window);
    add_border_to_aabb(pixel_bounds, Border(1));
    LOG_TRACE("Reading pixels [({}, {})-({}, {})] from dataset", pixel_bounds.min.x, pixel_bounds.min.y, pixel_bounds.max.x, pixel_bounds.max.y);
    auto height_map_result = reader.read_data_in_pixel_bounds_clamped(pixel_bounds);
    ALP_TRY(const HeightMap height_map, alp::wrap_error(std::move(height_map_result), UnreadableHeights{}));

    LOG_TRACE("Finding valid pixels");
    const float no_data_value = reader.get_no_data_value();
    const radix::RasterMask valid_mask = radix::raster::transform(height_map, [=](const float height) {
        return height != no_data_value;
    });

    LOG_TRACE("Transforming pixels to vertices");
    auto source_points_result = radix::raster::transform(height_map, valid_mask, [&](const float height, const glm::uvec2& coords) {
        return convert_pixel_to_vertex(height, coords, reader, pixel_bounds);
    });
    const auto source_points = *ASSERT_VAL(std::move(source_points_result), "the mask is made from the height map, so they have the same size");

    LOG_TRACE("Generating triangles");
    return meshify(source_points, valid_mask);
}

} // namespace

alp::Result<SimpleMesh, BuildMeshError> build_reference_mesh_patch(
    gis::Dataset& dataset,
    const gis::srs::System& mesh_srs,
    const gis::srs::System& clip_srs, const radix::geometry::Aabb3d& clip_bounds,
    const gis::srs::System& texture_srs, radix::tile::SrsBounds& texture_bounds) {
    const auto context = alp::make_context("building a mesh for {} in {} from {}", clip_bounds, clip_srs, dataset.name());

    auto source_srs_result = dataset.srs();
    ALP_TRY_CONTEXT(const gis::srs::System source_srs, alp::wrap_error(std::move(source_srs_result), UnsupportedDataset{}), context);

    // Coverage in the source srs is computed from ECEF bounds.
    const bool unbounded_height = !std::isfinite(clip_bounds.min.z) || !std::isfinite(clip_bounds.max.z);
    ALP_TRY_CONTEXT(const radix::geometry::Aabb3d ecef_clip_bounds, compute_ecef_clip_bounds(clip_srs, clip_bounds), context);

    // Find what data to read in the source srs.
    auto native_bounds_result = dataset.bounds();
    ALP_TRY_CONTEXT(const radix::tile::SrsBounds native_bounds, alp::wrap_error(std::move(native_bounds_result), UnsupportedDataset{}), context);
    auto regions_result = compute_native_regions(source_srs, native_bounds, ecef_clip_bounds).transform_error([&](ComputeNativeRegionsError error) {
        return std::move(error)
            .map<gis::srs::coverage::UnsupportedSystem, gis::srs::UnsupportedTransform, gis::srs::coverage::NotGeographic>(UnsupportedDataset{})
            .map<gis::srs::coverage::UnsupportedBounds>(UnsupportedClip{})
            .map<gis::UntransformablePoint>(OutOfBounds{});
    });
    ALP_TRY_CONTEXT(const std::vector<radix::tile::SrsBounds> regions, std::move(regions_result), context);
    // Each location is read at most once, so there is a seam at the edges of the turn centred on the native bounds.
    const std::vector<radix::tile::SrsBounds> windows = clip_to_native_bounds(regions, native_bounds);
    if (windows.empty()) {
        return alp::make_error(OutOfBounds{}, fmt::format("the regions {} lie outside the dataset bounds {}", fmt::join(regions, ", "), native_bounds)).context(context);
    }

    // Read height data in each window directly from dataset (no interpolation).
    auto reader_result = RawDatasetReader::for_dataset(dataset);
    ALP_TRY_CONTEXT(RawDatasetReader reader, alp::wrap_error(std::move(reader_result), UnsupportedDataset{}), context);
    std::vector<SimpleMesh> meshes_in_source_srs;
    for (const radix::tile::SrsBounds& window : windows) {
        ALP_TRY_CONTEXT(SimpleMesh mesh, read_heights_as_mesh(reader, window), context);
        // Check if we even have any valid vertices. Can happen if all of the region is padding.
        if (mesh.vertex_count() != 0 && mesh.face_count() != 0) {
            meshes_in_source_srs.push_back(std::move(mesh));
        }
    }
    if (meshes_in_source_srs.empty()) {
        return alp::make_error(EmptyRegion{}, "the valid heights in the windows form no triangle").context(context);
    }
    SimpleMesh mesh_in_source_srs = mesh::combine(meshes_in_source_srs);

    // Fast check if all vertices will be clipped. Horizontally, the windows already are close to
    // the clip bounds, so this only applies to their heights.
    if (!unbounded_height) {
        const radix::geometry::Aabb3d actual_source_bounds = calculate_bounds(mesh_in_source_srs);
        auto actual_ecef_bounds_result = gis::srs::coverage::compute_ecef(source_srs, actual_source_bounds).transform_error([&](gis::srs::coverage::Error error) {
            return std::move(error)
                .map<gis::srs::coverage::UnsupportedSystem>(UnsupportedDataset{})
                .map<gis::srs::coverage::UnsupportedBounds, gis::srs::TransformError>(UntransformableMesh{});
        });
        ALP_TRY_CONTEXT(const radix::geometry::Aabb3d actual_ecef_bounds, std::move(actual_ecef_bounds_result), context);
        if (!radix::geometry::intersect(actual_ecef_bounds, ecef_clip_bounds)) {
            return alp::make_error(EmptyRegion{}, "the heights lie outside the clip bounds").context(context);
        }
    }

    LOG_TRACE("Clipping mesh based on target bounds");
    ALP_TRY_CONTEXT(const SimpleMesh mesh_in_clip_srs, transform_mesh(std::move(mesh_in_source_srs), source_srs, clip_srs), context);
    SimpleMesh clipped_mesh = mesh::clip_on_bounds(mesh_in_clip_srs, clip_bounds);
    // Check if there are any vertices left
    if (clipped_mesh.vertex_count() == 0 || clipped_mesh.face_count() == 0) {
        return alp::make_error(EmptyRegion{}, "nothing is left after clipping").context(context);
    }

    // TODO: move this to another function?
    LOG_TRACE("Generating uv space and calculating required texture bounds");
    ALP_TRY_CONTEXT(clipped_mesh.uvs, generate_uv_space(clipped_mesh.positions, clip_srs, texture_srs, texture_bounds), context);

    LOG_TRACE("Transforming mesh into output srs");
    ALP_TRY_CONTEXT(SimpleMesh target_mesh, transform_mesh(std::move(clipped_mesh), clip_srs, mesh_srs), context);

    mesh::remove_isolated_vertices(target_mesh); // TODO: is this still required?
    mesh::debug_assert_valid(target_mesh);
    return target_mesh;
}

namespace {
std::string format_secs_since(const std::chrono::high_resolution_clock::time_point &start) {
    const auto duration = std::chrono::high_resolution_clock::now() - start;
    const double seconds = std::chrono::duration<double>(duration).count();
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << seconds;
    return ss.str();
}
}

alp::Result<SimpleMesh, BuildPatchError> build_patch(
    gis::Dataset &dataset,
    const gis::srs::System &target_bounds_srs,
    const radix::geometry::Aabb3d &target_bounds,
    const TileProvider* tile_provider,
    const gis::srs::System &mesh_srs) {
    const gis::srs::System texture_srs = gis::srs::webmercator();
    radix::tile::SrsBounds texture_bounds;

    std::chrono::high_resolution_clock::time_point start;
    start = std::chrono::high_resolution_clock::now();
    LOG_INFO("Building mesh...");
    ALP_TRY(SimpleMesh mesh, build_reference_mesh_patch(
        dataset,
        mesh_srs,
        target_bounds_srs, target_bounds,
        texture_srs, texture_bounds));
    LOG_DEBUG("Mesh building took {}s", format_secs_since(start));
    LOG_INFO("Finished building mesh geometry");

    if (tile_provider != nullptr) {
        start = std::chrono::high_resolution_clock::now();
        LOG_INFO("Assembling mesh texture");
        ALP_TRY(std::optional<AssembledTexture> texture, assemble_texture_from_tiles(texture_bounds, *tile_provider));
        if (texture.has_value()) {
            // The bounds of the assembled texture differ from the requested texture_bounds.
            texture.value().remap_uvs(mesh.uvs);
            mesh.texture = texture.value().image;
        }
        LOG_DEBUG("Assembling mesh texture took {}s", format_secs_since(start));
        LOG_INFO("Finished assembling mesh texture");
    } else {
        LOG_INFO("Skipped assembling texture");
    }

    return mesh;
}

}
