#pragma once

#include <string>
#include <string_view>

#include <fmt/format.h>

#include "gdal/Dataset.h"
#include "srs/System.h"
#include "srs/error.h"

#include "error/Error.h"
#include "error/Result.h"
#include "fmt_impls.h"
#include "gdal/CplError.h"
#include "gdal/PixelTransform.h"
#include "mesh/SimpleMesh.h"
#include "border.h"

namespace terrainbuilder {

// A requested region that lies outside the dataset, both in the dataset's srs.
struct OutOfBounds {
    radix::tile::SrsBounds requested;
    radix::tile::SrsBounds available;

    bool operator==(const OutOfBounds &) const = default;
};

inline std::string format_as(const OutOfBounds &error) {
    return fmt::format("the requested region {} lies outside the dataset region {}", error.requested, error.available);
}

// A requested region that holds no terrain.
struct EmptyRegion {
    bool operator==(const EmptyRegion &) const = default;
};

inline std::string_view format_as(const EmptyRegion) {
    return "the requested region holds no terrain";
}

// A requested region the dataset holds no terrain for.
using NoTerrain = alp::Error<OutOfBounds, EmptyRegion>;

// An error from building a mesh from a height dataset.
using BuildMeshError = alp::Error<NoTerrain, gdal::CplError, gdal::PixelTransformError, srs::TransformError>;

/// Builds a mesh from the given height dataset.
[[nodiscard]]
alp::Result<SimpleMesh, BuildMeshError> build_reference_mesh_patch(
    gdal::Dataset &dataset,
    const srs::System &mesh_srs,
    const srs::System &clip_srs, const radix::geometry::Aabb3d &clip_bounds,
    const srs::System &texture_srs, radix::tile::SrsBounds &texture_bounds);
}
