#include <vector>

#include <fmt/core.h>

#include "convert.h"
    
glm::dvec3 convert::cgal2glm(Point3 point) {
    return glm::dvec3(point[0], point[1], point[2]);
}
glm::dvec2 convert::cgal2glm(Point2 point) {
    return glm::dvec2(point[0], point[1]);
}
Point3 convert::glm2cgal(glm::dvec3 point) {
    return Point3(point[0], point[1], point[2]);
}
Point2 convert::glm2cgal(glm::dvec2 point) {
    return Point2(point[0], point[1]);
}

SurfaceMesh convert::mesh2cgal(const TerrainMesh &mesh) {
    SurfaceMesh cgal_mesh;

    for (const glm::dvec3 &position : mesh.positions) {
        const CGAL::SM_Vertex_index vertex = cgal_mesh.add_vertex(glm2cgal(position));
        assert(vertex != SurfaceMesh::null_vertex());
    }

    for (const glm::uvec3 &triangle : mesh.triangles) {
        CGAL::SM_Face_index face = cgal_mesh.add_face(
            CGAL::SM_Vertex_index(triangle.x),
            CGAL::SM_Vertex_index(triangle.y),
            CGAL::SM_Vertex_index(triangle.z));

        if (face == SurfaceMesh::null_face()) {
            fmt::println("Adding face failed, trying again with different order");
            face = cgal_mesh.add_face(
                CGAL::SM_Vertex_index(triangle.x),
                CGAL::SM_Vertex_index(triangle.z),
                CGAL::SM_Vertex_index(triangle.y));
        }
        assert(face != SurfaceMesh::null_face());
    }

#if DEBUG
    assert(cgal_mesh.is_valid(true));
    assert(CGAL::is_triangle_mesh(cgal_mesh));
#endif

    return cgal_mesh;
}
TerrainMesh convert::cgal2mesh(const SurfaceMesh &cgal_mesh) {
    TerrainMesh mesh;

    const size_t vertex_count = CGAL::num_vertices(cgal_mesh);
    const size_t face_count = CGAL::num_faces(cgal_mesh);
    mesh.positions.resize(vertex_count);
    mesh.uvs.resize(vertex_count);
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

cv::Mat convert::fi2cv(const FiImage &image) {
    std::vector<uint8_t> buffer = image.save_to_vector(FIF_PNG);
    cv::Mat mat = cv::imdecode(cv::Mat(1, buffer.size(), CV_8UC1, buffer.data()), cv::IMREAD_UNCHANGED);
    mat.convertTo(mat, CV_32FC3);
    return mat;
}

FiImage convert::cv2fi(const cv::Mat &image) {
    assert(!image.empty());

    cv::Mat converted;
    std::vector<uint8_t> buffer;
    image.convertTo(converted, CV_8UC3);
    cv::imencode(".png", image, buffer);
    return FiImage::load_from_buffer(buffer);
}
