#pragma once

#include <cstdint>
#include <memory>
#include <ogr_spatialref.h>

#include "tntn/geometrix.h"
#include "tntn/Mesh.h"
#include "tntn/File.h"

namespace tntn {

//exposed for testing
namespace detail {
using QMVertex = glm::dvec3;

struct QuantizedMeshHeader
{
  // The center of the tile in Earth-centered Fixed coordinates.
  // double CenterX;
  // double CenterY;
  // double CenterZ;
  QMVertex center;

  // The minimum and maximum heights in the area covered by this tile.
  // The minimum may be lower and the maximum may be higher than
  // the height of any vertex in this tile in the case that the min/max vertex
  // was removed during mesh simplification, but these are the appropriate
  // values to use for analysis or visualization.
  float MinimumHeight;
  float MaximumHeight;

  // The tile’s bounding sphere.  The X,Y,Z coordinates are again expressed
  // in Earth-centered Fixed coordinates, and the radius is in meters.
  QMVertex bounding_sphere_center;
  // double BoundingSphereCenterX;
  // double BoundingSphereCenterY;
  // double BoundingSphereCenterZ;
  double BoundingSphereRadius;

  // The horizon occlusion point, expressed in the ellipsoid-scaled Earth-centered Fixed frame.
  // If this point is below the horizon, the entire tile is below the horizon.
  // See http://cesiumjs.org/2013/04/25/Horizon-culling/ for more information.
  QMVertex horizon_occlusion;
  // double HorizonOcclusionPointX;
  // double HorizonOcclusionPointY;
  // double HorizonOcclusionPointZ;
};

uint16_t zig_zag_encode(int16_t i);
int16_t zig_zag_decode(uint16_t i);

} //namespace detail

detail::QuantizedMeshHeader quantised_mesh_header(const Mesh& m,
                                                  const BBox3D& bbox,
                                                  const OGRSpatialReference& srs,
                                                  bool mesh_is_rescaled = false);

bool write_mesh_as_qm(const char* filename, const Mesh& m, bool compress = false);
bool write_mesh_as_qm(const char* filename,
                      const Mesh& m,
                      const BBox3D& bbox,
                      bool mesh_is_rescaled = false,
                      bool compress = false);
bool write_mesh_as_qm(const char* filename,
                      const Mesh& m,
                      const BBox3D& bbox,
                      const OGRSpatialReference& mesh_srs,
                      bool mesh_is_rescaled = false,
                      bool compress = false
                      );

bool write_mesh_as_qm(const std::shared_ptr<FileLike>& f, const Mesh& m);
bool write_mesh_as_qm(const std::shared_ptr<FileLike>& f,
                      const Mesh& m,
                      const BBox3D& bbox,
                      const OGRSpatialReference& mesh_srs,
                      bool mesh_is_rescaled = false);

std::unique_ptr<Mesh> load_mesh_from_qm(const char* filename);
std::unique_ptr<Mesh> load_mesh_from_qm(const std::shared_ptr<FileLike>& f);

} // namespace tntn
