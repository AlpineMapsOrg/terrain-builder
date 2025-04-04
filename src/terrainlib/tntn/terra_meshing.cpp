#include "terra_meshing.h"
#include "MeshIO.h"
#include "TerraMesh.h"
#include "tntn_assert.h"

namespace tntn {

std::unique_ptr<Mesh> generate_tin_terra(std::unique_ptr<RasterDouble> raster, double max_error)
{
    TNTN_ASSERT(raster != nullptr);
    terra::TerraMesh g;
    g.load_raster(std::move(raster));
    g.greedy_insert(max_error);
    return g.convert_to_mesh();
}

std::unique_ptr<Mesh> generate_tin_terra(std::unique_ptr<SurfacePoints> surface_points,
    double max_error)
{
    auto raster = surface_points->to_raster();
    surface_points.reset();

    terra::TerraMesh g;
    g.load_raster(std::move(raster));
    g.greedy_insert(max_error);
    return g.convert_to_mesh();
}

std::unique_ptr<Mesh> generate_tin_terra(const SurfacePoints& surface_points, double max_error)
{
    auto raster = surface_points.to_raster();

    terra::TerraMesh g;
    g.load_raster(std::move(raster));
    g.greedy_insert(max_error);
    return g.convert_to_mesh();
}

} // namespace tntn
