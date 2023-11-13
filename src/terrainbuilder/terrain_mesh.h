#ifndef TERRAINMESH_H
#define TERRAINMESH_H

#include <vector>
#include <optional>
#include <opencv2/opencv.hpp>

class TerrainMesh {
public:
    std::vector<glm::uvec3> triangles;
    std::vector<glm::dvec3> positions;
    std::vector<glm::dvec2> uvs;
    std::optional<cv::Mat> texture;
};

#endif