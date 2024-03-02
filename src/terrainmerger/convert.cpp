#include <vector>

#include <fmt/core.h>

#include "convert.h"
#include "log.h"
#include "validate.h"
    
glm::dvec3 convert::cgal2glm(Point3 point) {
    return glm::dvec3(CGAL::to_double(point[0]), CGAL::to_double(point[1]), CGAL::to_double(point[2]));
}
glm::dvec2 convert::cgal2glm(Point2 point) {
    return glm::dvec2(CGAL::to_double(point[0]), CGAL::to_double(point[1]));
}
Point3 convert::glm2cgal(glm::dvec3 point) {
    return Point3(point[0], point[1], point[2]);
}
Point2 convert::glm2cgal(glm::dvec2 point) {
    return Point2(point[0], point[1]);
}

SurfaceMesh convert::mesh2cgal(const TerrainMesh &mesh) {
    LOG_TRACE("Converting terrain mesh to cgal mesh");

    SurfaceMesh cgal_mesh;

    for (const glm::dvec3 &position : mesh.positions) {
        const CGAL::SM_Vertex_index vertex = cgal_mesh.add_vertex(glm2cgal(position));
        assert(vertex != SurfaceMesh::null_vertex());
    }

    for (const glm::uvec3 &triangle : mesh.triangles) {
        const CGAL::SM_Face_index face = cgal_mesh.add_face(
            CGAL::SM_Vertex_index(triangle.x),
            CGAL::SM_Vertex_index(triangle.y),
            CGAL::SM_Vertex_index(triangle.z));
        assert(face != SurfaceMesh_::null_face());
    }

    return cgal_mesh;
}
TerrainMesh convert::cgal2mesh(const SurfaceMesh &cgal_mesh) {
    LOG_TRACE("Converting cgal mesh to terrain mesh");

    TerrainMesh mesh;

    const size_t vertex_count = CGAL::num_vertices(cgal_mesh);
    const size_t face_count = CGAL::num_faces(cgal_mesh);
    mesh.positions.resize(vertex_count);
    mesh.triangles.reserve(face_count);

    for (const CGAL::SM_Vertex_index vertex_index : cgal_mesh.vertices()) {
        const Point3 &position = cgal_mesh.point(vertex_index);
        mesh.positions[vertex_index] = cgal2glm(position);
    }

    for (const CGAL::SM_Face_index face_index : cgal_mesh.faces()) {
        glm::uvec3 triangle;
        unsigned int i = 0;
        for (const CGAL::SM_Vertex_index vertex_index : CGAL::vertices_around_face(cgal_mesh.halfedge(face_index), cgal_mesh)) {
            triangle[i] = vertex_index;
            i++;
        }
        mesh.triangles.push_back(triangle);
    }

    return mesh;
}

glm::dvec3 convert::cgal2glm_(Point3_ point) {
    return glm::dvec3(CGAL::to_double(point[0]), CGAL::to_double(point[1]), CGAL::to_double(point[2]));
}
glm::dvec2 convert::cgal2glm_(Point2_ point) {
    return glm::dvec2(CGAL::to_double(point[0]), CGAL::to_double(point[1]));
}
Point3_ convert::glm2cgal_(glm::dvec3 point) {
    return Point3_(point[0], point[1], point[2]);
}
Point2_ convert::glm2cgal_(glm::dvec2 point) {
    return Point2_(point[0], point[1]);
}

SurfaceMesh_ convert::mesh2cgal_(const TerrainMesh &mesh) {
    LOG_TRACE("Converting terrain mesh to cgal mesh");

    SurfaceMesh_ cgal_mesh;

    for (const glm::dvec3 &position : mesh.positions) {
        const CGAL::SM_Vertex_index vertex = cgal_mesh.add_vertex(glm2cgal_(position));
        assert(vertex != SurfaceMesh_::null_vertex());
    }

    for (const glm::uvec3 &triangle : mesh.triangles) {
        const CGAL::SM_Face_index face = cgal_mesh.add_face(
            CGAL::SM_Vertex_index(triangle.x),
            CGAL::SM_Vertex_index(triangle.y),
            CGAL::SM_Vertex_index(triangle.z));
        assert(face != SurfaceMesh_::null_face());
    }

    return cgal_mesh;
}

TerrainMesh convert::cgal2mesh_(const SurfaceMesh_ &cgal_mesh) {
    LOG_TRACE("Converting cgal mesh to terrain mesh");

    TerrainMesh mesh;

    const size_t vertex_count = CGAL::num_vertices(cgal_mesh);
    const size_t face_count = CGAL::num_faces(cgal_mesh);
    mesh.positions.resize(vertex_count);
    mesh.triangles.reserve(face_count);

    for (const CGAL::SM_Vertex_index vertex_index : cgal_mesh.vertices()) {
        const Point3_ &position = cgal_mesh.point(vertex_index);
        mesh.positions[vertex_index] = cgal2glm_(position);
    }

    for (const CGAL::SM_Face_index face_index : cgal_mesh.faces()) {
        glm::uvec3 triangle;
        unsigned int i = 0;
        for (const CGAL::SM_Vertex_index vertex_index : CGAL::vertices_around_face(cgal_mesh.halfedge(face_index), cgal_mesh)) {
            triangle[i] = vertex_index;
            i++;
        }
        mesh.triangles.push_back(triangle);
    }

    return mesh;
}
