#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Unique_hash_map.h>

#include "cgal.h"

inline size_t count_connected_components(const SurfaceMesh &mesh) {
    typedef CGAL::Unique_hash_map<FaceDescriptor, size_t> CcMap;
    typedef boost::associative_property_map<CcMap> CcPropertyMap;

    CcMap cc_map;
    CcPropertyMap cc_pmap(cc_map);
    const size_t num = CGAL::Polygon_mesh_processing::connected_components(mesh, cc_pmap);
    return num;
}

inline void validate_mesh(const SurfaceMesh& mesh) {
    assert(mesh.is_valid()); 
    assert(CGAL::is_triangle_mesh(mesh));
    assert(CGAL::is_valid_polygon_mesh(mesh));
    assert(count_connected_components(mesh) == 1);
    assert(!CGAL::Polygon_mesh_processing::does_self_intersect(mesh));
}

inline size_t count_connected_components(const SurfaceMesh_ &mesh) {
    typedef CGAL::Unique_hash_map<FaceDescriptor, size_t> CcMap;
    typedef boost::associative_property_map<CcMap> CcPropertyMap;

    CcMap cc_map;
    CcPropertyMap cc_pmap(cc_map);
    const size_t num = CGAL::Polygon_mesh_processing::connected_components(mesh, cc_pmap);
    return num;
}

inline void validate_mesh(const SurfaceMesh_& mesh) {
    assert(mesh.is_valid()); 
    assert(CGAL::is_triangle_mesh(mesh));
    assert(CGAL::is_valid_polygon_mesh(mesh));
    assert(count_connected_components(mesh) == 1);
    assert(!CGAL::Polygon_mesh_processing::does_self_intersect(mesh));
}
