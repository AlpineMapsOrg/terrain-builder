#pragma once

#include <bit>
#include <cstdint>

#include <glm/glm.hpp>

#include "geometry/bounds.h"
#include "numeric/wide_integer.h"

#include "bvh/NodeHeader.h"
#include "morton/code.h"

namespace morton {

// Counts the leading bits both codes share.
template <typename Code>
[[nodiscard]]
uint32_t count_common_prefix_bits(const Code &a, const Code &b) {
    const Code difference = a ^ b;
    if (difference == Code{0}) {
        return CODE_BITS;
    }

    // The branch not taken does not compile for this type, so it must not be instantiated.
    if constexpr (is_wide_integer_v<Code>) {
        return CODE_BITS - 1 - math::wide_integer::msb(difference);
    } else {
        return std::countl_zero(difference);
    }
}

// Morton code of `point` mapped into `scene`.
inline Code encode(const bvh::Bounds &scene, const glm::dvec3 &point) {
    constexpr CodeId::Level level = CodeId::max_level();
    const double max_coord = CodeId::max_coords_on_level(level).x;
    const glm::dvec3 relative_point = geometry::relativize_point(scene, point);
    const CodeId::Coords coords = relative_point * max_coord;
    return CodeId(level, coords).index_on_level();
}

// Morton code of the centre of `bounds`, mapped into `scene`.
inline Code encode(const bvh::Bounds &scene, const bvh::Bounds &bounds) {
    return encode(scene, bounds.centre());
}

// Morton code of a node's spatial position, taken at the centre of its tight bounds.
inline Code encode(const bvh::Bounds &scene, const bvh::NodeHeader &header) {
    return encode(scene, header.tight_bounds);
}

}
