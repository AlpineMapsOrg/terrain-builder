#pragma once

#include <filesystem>
#include <optional>

#include <radix/geometry.h>

#include "gdal/Dataset.h"
#include "error/Result.h"
#include "gdal/CplError.h"
#include "octree/Id.h"
#include "mesh/SimpleMesh.h"
#include "mesh/io/error.h"
#include "srs/System.h"
#include "srs/error.h"
#include "TileProvider.h"

namespace terrainbuilder {

// An error from building the mesh or texture of a patch.
using BuildPatchError = alp::Error<gdal::CplError, gdal::PixelTransformError, TileError, srs::TransformError>;

alp::Result<void, BuildPatchError, mesh::io::Error> build_and_save_patch(
    gdal::Dataset &dataset,
    const srs::System &target_bounds_srs,
    const radix::geometry::Aabb3d &target_bounds,
    const srs::System &texture_srs,
    const TileProvider *tile_provider,
    const srs::System &mesh_srs,
    const std::filesystem::path &output_path);

// The mesh for the target bounds, or nothing if the dataset holds no terrain there.
alp::Result<std::optional<SimpleMesh>, BuildPatchError> build_patch(
    gdal::Dataset &dataset,
    const srs::System &target_bounds_srs,
    const radix::geometry::Aabb3d &target_bounds,
    const srs::System &texture_srs,
    const TileProvider *tile_provider,
    const srs::System &mesh_srs);

alp::Result<void> build_all_patches(
    gdal::Dataset &dataset,
    const octree::Id::Level target_level,
    const srs::System &texture_srs,
    const TileProvider *tile_provider,
    const srs::System &mesh_srs,
    const std::filesystem::path &output_base_path,
    const std::string &output_format,
    const bool overwrite_existing);
}
