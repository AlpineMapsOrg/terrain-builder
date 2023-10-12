#ifndef TERRAINMESH_H
#define TERRAINMESH_H

class TerrainMesh {
public:
    std::vector<glm::uvec3> triangles;
    std::vector<glm::dvec3> positions;
    std::vector<glm::dvec2> uvs;
};

#endif