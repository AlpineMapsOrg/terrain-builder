#include <optional>
#include <vector>

#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_placement.h>
#include <CGAL/Surface_mesh_simplification/Edge_collapse_visitor_base.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>

#include <fmt/core.h>
#include <glm/glm.hpp>
#include <tl/expected.hpp>

#include "convert.h"
#include "simplify.h"
#include "uv_map.h"
#include "log.h"

// We use a different uv map type here, because we need this one to be attached to the mesh
// as otherwise the entries for removed vertices are not removed during garbage collection.
// We could use the same type as in uv_map but this would require a custom visitor or similar.
typedef SurfaceMesh::Property_map<VertexDescriptor, Point2> AttachedUvPropertyMap;

struct Uv_update_edge_collapse_visitor : CGAL::Surface_mesh_simplification::Edge_collapse_visitor_base<SurfaceMesh> {
    Uv_update_edge_collapse_visitor(AttachedUvPropertyMap uv_map)
        : uv_map(uv_map) {}

    // Called during the processing phase for each edge being collapsed.
    // If placement is absent the edge is left uncollapsed.
    void OnCollapsing(const Profile &profile,
                    boost::optional<Point> placement) {
        assert(!profile.is_v0_v1_a_border() && !profile.is_v1_v0_a_border());

        if (!placement) {
            return;
        }

        const VertexDescriptor v0 = profile.v0();
        const VertexDescriptor v1 = profile.v1();

        const glm::dvec3 pt = convert::cgal2glm(*placement);
        const glm::dvec3 p0 = convert::cgal2glm(profile.p0());
        const glm::dvec3 p1 = convert::cgal2glm(profile.p1());

        const auto w1 = std::clamp(glm::length(pt - p0) / glm::length(p1 - p0), 0.0, 1.0);
        const auto w0 = 1 - w1;
        
        const glm::dvec2 uv0 = convert::cgal2glm(get(uv_map, v0));
        const glm::dvec2 uv1 = convert::cgal2glm(get(uv_map, v1));
        this->new_uv = convert::glm2cgal(uv0 * w0 + uv1 * w1);
    }

    // Called after each edge has been collapsed
    void OnCollapsed(const Profile &, VertexDescriptor vd) {
        uv_map[vd] = new_uv;
    }

    AttachedUvPropertyMap uv_map;
    Point2 new_uv;
};

// Property map that indicates whether an edge is marked as non-removable.
struct Border_is_constrained_edge_map {
    const SurfaceMesh *mesh;
    typedef EdgeDescriptor key_type;
    typedef bool value_type;
    typedef value_type reference;
    typedef boost::readable_property_map_tag category;

    Border_is_constrained_edge_map(const SurfaceMesh &mesh)
        : mesh(&mesh) {}
    friend value_type get(const Border_is_constrained_edge_map &map, const key_type &edge) {
        return CGAL::is_border(edge, *map.mesh);
    }
};

typedef CGAL::Surface_mesh_simplification::Constrained_placement<CGAL::Surface_mesh_simplification::Midpoint_placement<SurfaceMesh>,
                                Border_is_constrained_edge_map> Placement;

void simplify_mesh_with_garland_heckbert(SurfaceMesh &mesh, AttachedUvPropertyMap uv_map) {
    /*
    typedef CGAL::Surface_mesh_simplification::GarlandHeckbert_plane_policies<Surface_mesh, Kernel> Classic_plane_policy;
    typedef CGAL::Surface_mesh_simplification::GarlandHeckbert_probabilistic_plane_policies<Surface_mesh, Kernel> Probabilistic_plane_policy;
    typedef CGAL::Surface_mesh_simplification::GarlandHeckbert_triangle_policies<Surface_mesh, Kernel> Classic_triangle_policy;
    typedef CGAL::Surface_mesh_simplification::GarlandHeckbert_probabilistic_triangle_policies<Surface_mesh, Kernel> Probabilistic_triangle_policy;
    */

    typedef CGAL::Surface_mesh_simplification::GarlandHeckbert_policies<SurfaceMesh, Kernel> GH_policies;
    typedef GH_policies::Get_cost GH_cost;
    typedef GH_policies::Get_placement GH_placement;
    typedef CGAL::Surface_mesh_simplification::Bounded_normal_change_placement<GH_placement> Bounded_GH_placement;

    GH_policies gh_policies(mesh);
    const GH_cost &gh_cost = gh_policies.get_cost();
    const GH_placement &gh_placement = gh_policies.get_placement();

    Border_is_constrained_edge_map bem(mesh);

    const CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<SurfaceMesh> stop(0.5);
    Uv_update_edge_collapse_visitor visitor(uv_map);
    const int _removed_edge_count = CGAL::Surface_mesh_simplification::edge_collapse(mesh, stop,
        CGAL::parameters::edge_is_constrained_map(bem)
            .get_placement(Placement(bem))
            .visitor(visitor));
}

cv::Mat simplify::simplify_texture(const cv::Mat& texture, glm::uvec2 target_resolution) {
    cv::Mat simplified_texture;
    cv::resize(texture, simplified_texture, cv::Size(target_resolution.x, target_resolution.y), cv::INTER_LINEAR);
    return simplified_texture;
}

void simplify::simplify_mesh_texture(TerrainMesh& mesh, glm::uvec2 target_resolution) {
    if (mesh.texture.has_value()) {
        mesh.texture = simplify_texture(mesh.texture.value(), target_resolution);
    }
}

TerrainMesh simplify::simplify_mesh(const TerrainMesh &mesh, Options options) {
    SurfaceMesh cgal_mesh = convert::mesh2cgal(mesh);

    AttachedUvPropertyMap uv_map = cgal_mesh.add_property_map<VertexDescriptor, Point2>("h:uv").first;
    for (size_t i = 0; i < mesh.uvs.size(); i++) {
        uv_map[CGAL::SM_Vertex_index(i)] = convert::glm2cgal(mesh.uvs[i]);
    }

    simplify_mesh_with_garland_heckbert(cgal_mesh, uv_map);

    cgal_mesh.collect_garbage();

    TerrainMesh simplified_mesh = convert::cgal2mesh(cgal_mesh);
    simplified_mesh.uvs.resize(simplified_mesh.vertex_count());
    for (size_t i = 0; i < CGAL::num_vertices(cgal_mesh); i++) {
        simplified_mesh.uvs[i] = convert::cgal2glm(uv_map[CGAL::SM_Vertex_index(i)]);
    }
    return simplified_mesh;

    /*
    const CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<SurfaceMesh> stop(options.stop_ratio);

    typedef CGAL::Surface_mesh_simplification::GarlandHeckbert_policies<SurfaceMesh, Kernel> GH_policies;
    typedef GH_policies::Get_cost GH_cost;
    typedef GH_policies::Get_placement GH_placement;
    typedef CGAL::Surface_mesh_simplification::Bounded_normal_change_placement<GH_placement> Bounded_GH_placement;
     

    bool check_mesh = CGAL::is_valid_polygon_mesh(cgal_mesh);
    std::cout << "vaild or in valid: " << check_mesh << std::endl;
    GH_policies gh_policies(cgal_mesh);
    const GH_cost& gh_cost = gh_policies.get_cost();
    const GH_placement& gh_placement = gh_policies.get_placement();
    Bounded_GH_placement placement(gh_placement);

    SM_pmap sm_pmap = cgal_mesh.add_property_map<vertex_descriptor, size_t>("v:sm").first;
    UV_pmap uv_pmap = cgal_mesh.add_property_map<vertex_descriptor, Point_2>("h:uv").first;
    My_visitor vis(sm_pmap, uv_pmap);
    for (unsigned int i = 0; i < final_uvs.size(); i++) {
        uv_pmap[CGAL::SM_Vertex_index(i)] = glm2cgal(final_uvs[i]);
    }
    for (unsigned int i = 0; i < new_mesh_id.size(); i++) {
        sm_pmap[CGAL::SM_Vertex_index(i)] = new_mesh_id[i];
    }

    std::cout << "Input mesh has " << CGAL::num_vertices(cgal_mesh) << " nv "<< CGAL::num_edges(cgal_mesh) << " ne "
        << CGAL::num_faces(cgal_mesh) << " nf" << std::endl;
    // int r = CGAL::Surface_mesh_simplification::edge_collapse(cgal_mesh, stop, CGAL::parameters::get_cost(gh_cost).get_placement(placement).visitor(vis)); 

    // Contract the surface mesh as much as possible
    // CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<SurfaceMesh> stop(stop_ratio);
    Border_is_constrained_edge_map bem(cgal_mesh);
    // This the actual call to the simplification algorithm.
    // The surface mesh and stop conditions are mandatory arguments.
    int r = CGAL::Surface_mesh_simplification::edge_collapse(cgal_mesh, stop, CGAL::parameters::edge_is_constrained_map(bem).get_placement(Placement(bem)).visitor(vis));

    TerrainMesh final_mesh;
    const size_t vertex_count = CGAL::num_vertices(cgal_mesh);
    const size_t face_count = CGAL::num_faces(cgal_mesh);
    // final_mesh.positions.resize(vertex_count);
    final_mesh.positions.reserve(vertex_count);
    final_mesh.uvs.reserve(vertex_count);
    final_mesh.triangles.resize(face_count);

    std::vector<unsigned int> mapmapmap;
    mapmapmap.resize(vertex_count);

    for (const CGAL::SM_Vertex_index vertex_index : cgal_mesh.vertices()) {
        const Point_3 &vertex = cgal_mesh.point(vertex_index);
        mapmapmap[vertex_index] = final_mesh.positions.size();
        // final_mesh.positions[vertex_index] = glm::dvec3(vertex[0], vertex[1], vertex[2]) + average_position;
        final_mesh.positions.push_back(glm::dvec3(vertex[0], vertex[1], vertex[2]) + average_position);
    }
    for (const glm::dvec3 position : final_mesh.positions) {
        assert(position.x != 0);
    }
    for (const CGAL::SM_Face_index face_index : cgal_mesh.faces()) {
        std::array<unsigned int, 3> triangle;
        unsigned int i = 0;
        for (const CGAL::SM_Vertex_index vertex_index : CGAL::vertices_around_face(cgal_mesh.halfedge(face_index), cgal_mesh)) {
            triangle[i] =  mapmapmap[vertex_index];
            i++;
        }
        final_mesh.triangles.emplace_back(triangle[0], triangle[1], triangle[2]);
    }
    for (const CGAL::SM_Vertex_index vertex_index : cgal_mesh.vertices()) {
        // final_mesh.uvs[vertex_index] = glm::dvec2(0, 0);
    }
    for (const CGAL::SM_Vertex_index vertex_index : cgal_mesh.vertices()) {
        const glm::dvec2 &uv = cgal2glm(uv_pmap[vertex_index]);
        final_mesh.uvs.push_back((uv - glm::dvec2(0.5)) * 0.99 + glm::dvec2(0.5));
    }

    std::vector<glm::vec3> colors;
    for (const CGAL::SM_Vertex_index vertex_index : cgal_mesh.vertices()) {
        const size_t sm = sm_pmap[vertex_index];
        colors.emplace_back(static_cast<float>(sm)/4);
    }
    */
}
