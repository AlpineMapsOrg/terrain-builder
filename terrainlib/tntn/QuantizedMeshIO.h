#pragma once

#include <cstdint>
#include <memory>
#include <ogr_spatialref.h>

#include "tntn/File.h"
#include "tntn/Mesh.h"
#include "tntn/geometrix.h"

namespace tntn {

// exposed for testing
namespace detail {
    using QMVertex = glm::dvec3;
    using VertexOrdering = std::unordered_map<Vertex, unsigned int>;

    struct QuantizedMeshHeader {
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

    struct QuantizedMeshVertexData {
        std::vector<uint32_t> northlings;
        std::vector<uint32_t> eastlings;
        std::vector<uint32_t> southlings;
        std::vector<uint32_t> westlings;
        VertexOrdering vertices_order;
        std::vector<uint16_t> us;
        std::vector<uint16_t> vs;
        std::vector<uint16_t> hs;
        size_t nvertices;
    };

    uint16_t zig_zag_encode(int16_t i);
    int16_t zig_zag_decode(uint16_t i);

    std::vector<uint16_t> quantized_mesh_encode(const std::vector<int16_t>& input);
    std::vector<int16_t> quantized_mesh_decode(const std::vector<uint16_t>& input);

    QuantizedMeshHeader quantised_mesh_header(const Mesh& m,
        const BBox3D& bbox,
        const OGRSpatialReference& srs,
        bool mesh_is_rescaled = false);

    QuantizedMeshVertexData quantised_mesh_vertex_data(const Mesh& m,
        const BBox3D& bbox,
        const OGRSpatialReference& srs,
        bool mesh_is_rescaled = false);

} // namespace detail

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
    bool compress = false);

bool write_mesh_as_qm(const std::shared_ptr<FileLike>& f, const Mesh& m);
bool write_mesh_as_qm(const std::shared_ptr<FileLike>& f,
    const Mesh& m,
    const BBox3D& bbox,
    const OGRSpatialReference& mesh_srs,
    bool mesh_is_rescaled = false);

std::unique_ptr<Mesh> load_mesh_from_qm(const char* filename);
std::unique_ptr<Mesh> load_mesh_from_qm(const std::shared_ptr<FileLike>& f);

} // namespace tntn
