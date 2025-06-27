#pragma once
#include <glm/glm.hpp>
#include <radix/geometry.h>

namespace radix::geometry
{
    // https://en.wikipedia.org/wiki/Slab_method#Algorithm
    template <typename T>
    glm::vec<2, T> ray_intersect(const Aabb<3, T> &a, const glm::vec<3, T> &ray_origin, const glm::vec<3, T> &ray_direction)
    {
        glm::vec<3, T> t_low = (a.min - ray_origin) / ray_direction;
        glm::vec<3, T> t_high = (a.max - ray_origin) / ray_direction;

        glm::vec<3, T> t_close_v = glm::min(t_low, t_high);
        glm::vec<3, T> t_far_v = glm::max(t_low, t_high);

        T t_close = glm::max(glm::max(t_close_v.x, t_close_v.y), t_close_v.z);
        T t_far = glm::min(glm::min(t_far_v.x, t_far_v.y), t_far_v.z);

        return glm::vec<2, T>(t_close, t_far);
    }
}