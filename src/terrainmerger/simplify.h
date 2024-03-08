#ifndef SIMPLIFY_H
#define SIMPLIFY_H

#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

#include "cgal.h"
#include "mesh/terrain_mesh.h"

using namespace kernel::epick;
namespace simplify {
// We use a different uv map type here, because we need this one to be attached to the mesh
// as otherwise the entries for removed vertices are not removed during garbage collection.
// We could use the same type as in uv_map but this would require a custom visitor or similar.
typedef SurfaceMesh::Property_map<VertexDescriptor, Point2> AttachedUvPropertyMap;

cv::Mat simplify_texture(const cv::Mat& texture, glm::uvec2 target_resolution);

void simplify_mesh_texture(TerrainMesh& mesh, glm::uvec2 target_resolution);

enum class Algorithm {
    GarlandHeckbert,
    LindstromTurk
};

struct EdgeRatio {
    double ratio = 1;
};
struct AbsoluteError {
    double error_bound;
};
struct RelativeError {
    double error_bound;
};

using StopCondition = std::variant<EdgeRatio, AbsoluteError, RelativeError>;

struct Options {
    Algorithm algorithm = Algorithm::LindstromTurk;
    bool lock_borders = true;
};

struct Result {
    TerrainMesh mesh;
    double max_absolute_error;
};

Result simplify_mesh(const TerrainMesh &mesh, StopCondition stop_condition, Options options = Options());

}

// fmt support
template <> struct fmt::formatter<simplify::Algorithm>: formatter<string_view> {
    auto format(const simplify::Algorithm& algorithm, format_context& ctx) const;
};

#endif
