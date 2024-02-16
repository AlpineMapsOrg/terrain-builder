#ifndef CONVERT_H
#define CONVERT_H

#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>

#include "mesh/terrain_mesh.h"
#include "cgal.h"

namespace convert {

glm::dvec3 cgal2glm(Point3 point);
glm::dvec2 cgal2glm(Point2 point);
Point3 glm2cgal(glm::dvec3 point);
Point2 glm2cgal(glm::dvec2 point);

SurfaceMesh mesh2cgal(const TerrainMesh& mesh);
TerrainMesh cgal2mesh(const SurfaceMesh& cgal_mesh);

}

#endif
