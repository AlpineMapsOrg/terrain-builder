#pragma once

#include <glm/glm.hpp>
#include <radix/geometry.h>
#include <zpp_bits.h>

#include "glm_utils.h"

namespace zpp::bits {

template <typename Archive, glm::length_t L, typename T, glm::qualifier Q>
auto serialize(Archive &archive, const glm::vec<L, T, Q> &v) {
    return archive(as_span(v));
}

template <typename Archive, glm::length_t L, typename T, glm::qualifier Q>
auto serialize(Archive &archive, glm::vec<L, T, Q> &v) {
    return archive(as_span(v));
}

template <typename Archive, glm::length_t L, typename T>
auto serialize(Archive &archive, const radix::geometry::Aabb<L, T> &bounds) {
    return archive(bounds.min, bounds.max);
}

template <typename Archive, glm::length_t L, typename T>
auto serialize(Archive &archive, radix::geometry::Aabb<L, T> &bounds) {
    return archive(bounds.min, bounds.max);
}

}