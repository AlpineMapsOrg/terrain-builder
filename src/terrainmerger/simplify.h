#ifndef SIMPLIFY_H
#define SIMPLIFY_H

#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

#include "cgal.h"
#include "fi_image.h"
#include "terrain_mesh.h"

namespace simplify {
// We use a different uv map type here, because we need this one to be attached to the mesh
// as otherwise the entries for removed vertices are not removed during garbage collection.
// We could use the same type as in uv_map but this would require a custom visitor or similar.
typedef SurfaceMesh::Property_map<VertexDescriptor, Point2> AttachedUvPropertyMap;

cv::Mat simplify_texture(const cv::Mat& texture, glm::uvec2 target_resolution);

FiImage simplify_texture(const FiImage& texture, glm::uvec2 target_resolution);

void simplify_mesh_texture(TerrainMesh& mesh, glm::uvec2 target_resolution);

enum class Algorithm {
    GarlandHeckbert
};

struct Options {
    Algorithm algorithm;
    bool lock_borders;
    double stop_ratio;

    Options() : algorithm(Algorithm::GarlandHeckbert), lock_borders(true), stop_ratio(0.25) {}
};

TerrainMesh simplify_mesh(const TerrainMesh &mesh, Options options = Options());

}

#endif
