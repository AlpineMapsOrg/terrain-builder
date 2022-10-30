#pragma once

#include "tntn/Mesh.h"
#include "tntn/geometrix.h"
#include <memory>

namespace tntn {
bool generate_delaunay_faces(const std::vector<Vertex>& vlist, std::vector<Face>& faces);
std::unique_ptr<Mesh> generate_delaunay_mesh(std::vector<Vertex>&& vlist);

} // namespace tntn
