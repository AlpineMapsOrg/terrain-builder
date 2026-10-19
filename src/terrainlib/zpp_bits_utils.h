#pragma once

#include <concepts>
#include <system_error>
#include <type_traits>

#include <glm/glm.hpp>
#include <radix/geometry.h>
#include <zpp_bits.h>

#include "glm_utils.h"

namespace zpp::bits {

// The result of an archive that serializes.
template <typename R>
concept ArchiveResult = std::same_as<std::remove_cvref_t<R>, errc> || std::same_as<std::remove_cvref_t<R>, std::errc>;

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
