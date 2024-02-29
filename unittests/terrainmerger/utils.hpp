#ifndef UTILS_HPP
#define UTILS_HPP

#include <algorithm>
#include <vector>

#include <glm/glm.hpp>

inline void sort_triangles(std::vector<glm::uvec3>& triangles) {
    std::sort(triangles.begin(), triangles.end(),
              [](const glm::uvec3 a, const glm::uvec3 b) { return a.x + a.y + a.z < b.x + b.y + b.z; });

    for (glm::uvec3& triangle : triangles) {
        unsigned int min_index = 0;
        for (size_t k = 1; k < static_cast<size_t>(triangle.length()); k++) {
            if (triangle[min_index] > triangle[k]) {
                min_index = k;
            }
        }
        if (min_index == 0) {
            continue;
        }

        glm::uvec3 new_triangle;
        for (size_t k = 0; k < static_cast<size_t>(triangle.length()); k++) {
            new_triangle[k] = triangle[(min_index + k) % triangle.length()];
        }

        triangle = new_triangle;
    }
}

#endif
