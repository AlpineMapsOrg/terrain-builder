#pragma once

#include "bvh/NodeHeader.h"
#include "octree/Space.h"

namespace bvh {

constexpr bvh::Bounds earth_bounds() {
    return octree::Space::earth().bounds();
}

}
