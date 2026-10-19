#pragma once

#include "error/Result.h"
#include "mesh/SimpleMesh.h"
#include "mesh/non_manifold_result.h"

namespace mesh {

struct IntersectionAndDifference {
    SimpleMesh intersection;
    SimpleMesh difference; // a - b
};
[[nodiscard]]
alp::Result<IntersectionAndDifference, NonManifoldResult> intersection_and_difference(const SimpleMesh &a, const SimpleMesh &b);

}
