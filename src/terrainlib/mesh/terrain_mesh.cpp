#include <ranges>

#include "terrain_mesh.h"

geometry::Aabb<3, double> calculate_bounds(const TerrainMesh& mesh) {
    geometry::Aabb<3, double> bounds;
    bounds.min = glm::dvec3(std::numeric_limits<double>::infinity());
    bounds.max = glm::dvec3(-std::numeric_limits<double>::infinity());
    for (unsigned int j = 0; j < mesh.positions.size(); j++) {
        const auto &position = mesh.positions[j];
        bounds.expand_by(position);
    }
    return bounds;
}

geometry::Aabb<3, double> calculate_bounds(std::span<const TerrainMesh> meshes) {
    geometry::Aabb<3, double> bounds;
    bounds.min = glm::dvec3(std::numeric_limits<double>::infinity());
    bounds.max = glm::dvec3(-std::numeric_limits<double>::infinity());
    for (unsigned int i = 0; i < meshes.size(); i++) {
        const TerrainMesh &mesh = meshes[i];
        for (unsigned int j = 0; j < mesh.positions.size(); j++) {
            const auto &position = mesh.positions[j];
            bounds.expand_by(position);
        }
    }
    return bounds;
}

std::vector<size_t> find_isolated_vertices(const TerrainMesh& mesh) {
    std::vector<bool> connected;
    connected.resize(mesh.vertex_count());
    std::fill(connected.begin(), connected.end(), false);
    for (const glm::uvec3 &triangle : mesh.triangles) {
        for (size_t k = 0; k < static_cast<size_t>(triangle.length()); k++) {
            connected[triangle[k]] = true;
        }
    }

    std::vector<size_t> isolated;
    for (size_t i = 0; i < mesh.vertex_count(); i++) {
        if (!connected[i]) {
            isolated.push_back(i);
        }
    }

    return isolated;
}

size_t remove_isolated_vertices(TerrainMesh& mesh) {
    const bool has_uvs = mesh.has_uvs();
    const std::vector<size_t> isolated = find_isolated_vertices(mesh);

    std::vector<size_t> index_offset;
    for (size_t i : isolated | std::views::reverse) {
        const size_t last_index = mesh.positions.size() - 1;
        std::swap(mesh.positions[i], mesh.positions[last_index]);
        mesh.positions.pop_back();
        if (has_uvs) {
            std::swap(mesh.uvs[i], mesh.uvs[last_index]);
        }
        mesh.uvs.pop_back();

        for (glm::uvec3 &triangle : mesh.triangles) {
            for (size_t k = 0; k < static_cast<size_t>(triangle.length()); k++) {
                if (triangle[k] == last_index) {
                    triangle[k] = i;
                }
            }
        }
    }

    return isolated.size();
}

size_t remove_triangles_of_negligible_size(TerrainMesh& mesh, const double threshold_percentage_of_average) {
    std::vector<double> areas;
    areas.reserve(mesh.triangles.size());
    for (glm::uvec3 &triangle : mesh.triangles) {
        const std::array<glm::dvec3, triangle.length()> points{
            mesh.positions[triangle.x],
            mesh.positions[triangle.y],
            mesh.positions[triangle.z]};

        // const double area = Kernel().compute_area_3_object()(cgal_points[0], cgal_points[1], cgal_points[2]);
        const double area = 0.5 * std::abs(
                                      points[0].x * (points[1].y - points[2].y) +
                                      points[1].x * (points[2].y - points[0].y) +
                                      points[2].x * (points[0].y - points[1].y));

        areas.push_back(area);
    }

    const double average_area = std::reduce(areas.begin(), areas.end()) / static_cast<double>(areas.size());
    const size_t erased_count = std::erase_if(mesh.triangles, [&](const glm::uvec3 &triangle) {
        const size_t index = &triangle - &*mesh.triangles.begin();
        const double area = areas[index];
        return area < average_area * threshold_percentage_of_average;
    });

    return erased_count;
}

void sort_triangles(TerrainMesh& mesh) {
    sort_triangles(mesh.triangles);
}

void sort_triangles(std::span<glm::uvec3> triangles) {
    // sort vertices in triangles
    for (glm::uvec3 &triangle : triangles) {
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

    // sort triangle vector
    std::sort(triangles.begin(), triangles.end(), [](const glm::uvec3 a, const glm::uvec3 b) {
        // First, compare by x
        if (a.x != b.x) {
            return a.x < b.x;
        }

        // If x is equal, compare by y
        if (a.y != b.y) {
            return a.y < b.y;
        }

        // If x and y are equal, compare by z
        return a.z < b.z;
    });
}

static void validate_sorted_mesh(const TerrainMesh &mesh) {
    // check correct count of uvs
    assert(!mesh.has_uvs() || mesh.positions.size() == mesh.uvs.size());
    
    // check uvs between 0 and 1
    for (const glm::dvec2 &uv : mesh.uvs) {
        for (size_t k = 0; k < static_cast<size_t>(uv.length()); k++) {
            assert(uv[k] >= 0);
            assert(uv[k] <= 1);
        }
    }

    // check for vertex indices in triangles outside valid range
    for (const glm::uvec3 &triangle : mesh.triangles) {
        for (size_t k = 0; k < static_cast<size_t>(triangle.length()); k++) {
            const size_t vertex_index = triangle[k];
            assert(vertex_index < mesh.vertex_count());
        }
    }

    // check for degenerate triangles
    for (const glm::uvec3 &triangle : mesh.triangles) {
        assert(triangle.x != triangle.y);
        assert(triangle.y != triangle.z);
    }

    // check for duplicated triangles
    assert(mesh.triangles.end() == std::adjacent_find(mesh.triangles.begin(), mesh.triangles.end()));

    // check for duplicated triangles with different orientation
    std::vector<glm::uvec3> triangles_ignore_orientation(mesh.triangles);
    // sort vertices in triangles
    for (glm::uvec3 &triangle : triangles_ignore_orientation) {
        std::sort(&triangle.x, &triangle.z);
    }
    std::vector<glm::uvec3> triangles_ignore_orientation2(triangles_ignore_orientation);
    sort_triangles(triangles_ignore_orientation);
    assert(triangles_ignore_orientation.end() == std::adjacent_find(triangles_ignore_orientation.begin(), triangles_ignore_orientation.end()));

    // check for isolated vertices
    assert(find_isolated_vertices(mesh).empty());
}

void validate_mesh(const TerrainMesh &mesh) {
#if NDEBUG
    return;
#endif
    TerrainMesh sorted(mesh);
    sort_triangles(sorted);
    validate_sorted_mesh(sorted);
}
