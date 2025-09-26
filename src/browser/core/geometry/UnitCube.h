#pragma once
#include <glm/glm.hpp>
#include <vector>

class UnitCube
{
public:
    static constexpr std::vector<glm::vec3> vertices()
    {
        return {
            glm::vec3(-0.5, -0.5, -0.5), // 000  //#0
            glm::vec3(-0.5, 0.5, -0.5),  // 010  //#1
            glm::vec3(0.5, 0.5, -0.5),   // 110  //#2
            glm::vec3(0.5, -0.5, -0.5),  // 100  //#3

            glm::vec3(-0.5, -0.5, 0.5), // 001   //#4
            glm::vec3(-0.5, 0.5, 0.5),  // 011   //#5
            glm::vec3(0.5, 0.5, 0.5),   // 111   //#6
            glm::vec3(0.5, -0.5, 0.5),  // 101   //#7
        };
    }

    static constexpr std::vector<unsigned int> line_indices()
    {
        return {
            // Front Loop
            0, 1, 1, 2, 2, 3, 3, 0,
            // Back Loop
            4, 5, 5, 6, 6, 7, 7, 4,
            // Connecting Front and Back
            0, 4,
            1, 5,
            2, 6,
            3, 7};
    }

    static constexpr std::vector<unsigned int> mesh_indices()
    {
        return {
            // Front Face
            1, 3, 0, // Lower
            2, 3, 1, // Upper
            // Back Face
            6, 4, 7, // Lower
            5, 4, 6, // Upper
            // Top Face
            5, 2, 1, // Lower
            6, 2, 5, // Upper
            // Bottom Face
            0, 7, 4, // Lower
            3, 7, 0, // Upper
            // Left Face
            5, 0, 4, // Lower
            1, 0, 5, // Upper
            // Right Face
            2, 7, 3, // Lower
            6, 7, 2, // Upper
        };
    }
};