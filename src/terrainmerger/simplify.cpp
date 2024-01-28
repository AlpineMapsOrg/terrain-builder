#include <optional>
#include <vector>

#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_placement.h>
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
    typedef EdgeDescriptor key_type;
    typedef bool value_type;
    typedef value_type reference;
    typedef boost::readable_property_map_tag category;

    const SurfaceMesh *mesh;
    const bool active = true;

    Border_is_constrained_edge_map(const SurfaceMesh &mesh, const bool active = true)
        : mesh(&mesh), active(active) {}
        
    friend value_type get(const Border_is_constrained_edge_map &map, const key_type &edge) {
        return map.active && CGAL::is_border(edge, *map.mesh);
    }

};

template <class Cost, class Placement>
static void _simplify_mesh(
    SurfaceMesh &mesh,
    AttachedUvPropertyMap &uv_map,
    const Cost &cost,
    const Placement &placement,
    const bool lock_borders = true) {
    typedef typename CGAL::Surface_mesh_simplification::Constrained_placement<Placement, Border_is_constrained_edge_map> ConstrainedPlacement;

    const size_t initial_vertex_count = CGAL::num_vertices(mesh);
    const size_t initial_edge_count = CGAL::num_edges(mesh);
    const size_t initial_face_count = CGAL::num_faces(mesh);

    Border_is_constrained_edge_map bem(mesh, lock_borders);

    const ConstrainedPlacement constrained_placement(bem, placement);

    const CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<SurfaceMesh> stop(0.25);
    Uv_update_edge_collapse_visitor visitor(uv_map);

    const int removed_edge_count = CGAL::Surface_mesh_simplification::edge_collapse(mesh, stop,
        CGAL::parameters::edge_is_constrained_map(bem)
            .get_placement(constrained_placement)
            .get_cost(cost)
            .visitor(visitor));
    LOG_TRACE("Removed {} edges from simplified mesh", removed_edge_count);

    const size_t simplified_vertex_count = CGAL::num_vertices(mesh);
    const size_t simplified_edge_count = CGAL::num_edges(mesh);
    const size_t simplified_face_count = CGAL::num_faces(mesh);
    LOG_DEBUG("Simplified mesh to {}/{} vertices, {}/{} edges, {}/{} faces",
              simplified_vertex_count, initial_vertex_count,
              simplified_edge_count, initial_edge_count,
              simplified_face_count, initial_face_count);

    // Actually remove the vertices, edges and faces
    mesh.collect_garbage();
}

template <class Policies>
static void _simplify_mesh(
    SurfaceMesh &mesh,
    AttachedUvPropertyMap& uv_map,
    const Policies& policies,
    const bool lock_borders = true) {
    typedef typename Policies::Get_cost Cost;
    typedef typename Policies::Get_placement Placement;

    const Cost &cost = policies.get_cost();
    const Placement &placement = policies.get_placement();

    _simplify_mesh(mesh, uv_map, cost, placement, lock_borders);
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

    switch (options.algorithm) {
    case Algorithm::GarlandHeckbert:
        typedef CGAL::Surface_mesh_simplification::GarlandHeckbert_policies<SurfaceMesh, Kernel> GH_policies;
        _simplify_mesh<GH_policies>(cgal_mesh, uv_map, GH_policies(cgal_mesh));
        break;
    case Algorithm::LindstromTurk:
        typedef CGAL::Surface_mesh_simplification::LindstromTurk_cost<SurfaceMesh> LT_cost;
        typedef CGAL::Surface_mesh_simplification::LindstromTurk_placement<SurfaceMesh> LT_placement;
        _simplify_mesh<LT_cost, LT_placement>(cgal_mesh, uv_map, LT_cost(), LT_placement());
        break;
    }

    TerrainMesh simplified_mesh = convert::cgal2mesh(cgal_mesh);
    simplified_mesh.uvs.resize(simplified_mesh.vertex_count());
    for (size_t i = 0; i < CGAL::num_vertices(cgal_mesh); i++) {
        simplified_mesh.uvs[i] = convert::cgal2glm(uv_map[CGAL::SM_Vertex_index(i)]);
    }
    return simplified_mesh;
}
