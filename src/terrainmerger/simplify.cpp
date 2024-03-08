#include <optional>
#include <vector>

// required before distance.h due to a bug in cgal
// https://github.com/CGAL/cgal/issues/8009
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>

#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/Polygon_mesh_processing/repair_self_intersections.h>
#include <CGAL/Surface_mesh_simplification/Edge_collapse_visitor_base.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_placement.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/tags.h>

#include <fmt/core.h>
#include <fmt/format.h>
#include <glm/glm.hpp>
#include <radix/geometry.h>
#include <tl/expected.hpp>

#include "convert.h"
#include "log.h"
#include "simplify.h"
#include "uv_map.h"
#include "validate.h"

using namespace simplify;

template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};

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
        // only valid if restrict_border_triangles=false
        // assert(!profile.is_v0_v1_a_border() && !profile.is_v1_v0_a_border());

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

auto fmt::formatter<Algorithm>::format(const Algorithm &algorithm, format_context &ctx) const {
    string_view name = "unknown";
    switch (algorithm) {
    case Algorithm::GarlandHeckbert:
        name = "GarlandHeckbert";
        break;
    case Algorithm::LindstromTurk:
        name = "LindstromTurk";
        break;
    }
    return formatter<string_view>::format(name, ctx);
}

std::ostream &operator<<(std::ostream &os, const Algorithm &algorithm) {
    os << fmt::format("{}", algorithm);
    return os;
}

// Property map that indicates whether an edge is marked as non-removable.
struct Border_is_constrained_edge_map {
    typedef EdgeDescriptor key_type;
    typedef bool value_type;
    typedef value_type reference;
    typedef boost::readable_property_map_tag category;

    const SurfaceMesh *mesh;
    const bool active = true;
    const bool restrict_border_triangles = true;

    Border_is_constrained_edge_map(const SurfaceMesh &mesh, const bool active = true)
        : mesh(&mesh), active(active) {}

    friend value_type get(const Border_is_constrained_edge_map &map, const key_type &edge) {
        if (!map.active) {
            return false;
        }

        const SurfaceMesh &mesh = *map.mesh;

        // return CGAL::is_border(edge, mesh); // old version
        if (CGAL::is_border(edge, mesh)) {
            return true;
        }

        if (!map.restrict_border_triangles) {
            return false;
        }

        const VertexDescriptor v0 = mesh.vertex(edge, 0);
        const VertexDescriptor v1 = mesh.vertex(edge, 1);
        if (CGAL::is_border(v0, mesh).has_value() || CGAL::is_border(v1, mesh).has_value()) {
            return true;
        }

        return false;
    }
};

struct SimplificationArgs {
    bool lock_borders;
    double stop_edge_ratio;
};

template <class Cost, class Placement>
static size_t _simplify_mesh_with_cost_and_placement(
    SurfaceMesh &mesh,
    AttachedUvPropertyMap &uv_map,
    const Cost &cost,
    const Placement &placement,
    const SimplificationArgs args) {
    if (args.stop_edge_ratio > 0.999) {
        LOG_DEBUG("Skipped simplification due to stop ratio being almost 100% ({:g})", args.stop_edge_ratio);
        return 0;
    }

    typedef typename CGAL::Surface_mesh_simplification::Constrained_placement<Placement, Border_is_constrained_edge_map> ConstrainedPlacement;
    Border_is_constrained_edge_map bem(mesh, args.lock_borders);
    const ConstrainedPlacement constrained_placement(bem, placement);

    const CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<SurfaceMesh> stop_predicate(args.stop_edge_ratio);
    Uv_update_edge_collapse_visitor visitor(uv_map);

    const size_t removed_edge_count = CGAL::Surface_mesh_simplification::edge_collapse(mesh, stop_predicate,
                                                                                       CGAL::parameters::edge_is_constrained_map(bem)
                                                                                           .get_placement(constrained_placement)
                                                                                           .get_cost(cost)
                                                                                           .visitor(visitor));

    LOG_TRACE("Removed {} edges from simplified mesh", removed_edge_count);

    // Actually remove the vertices, edges and faces
    mesh.collect_garbage();

    return removed_edge_count;
}

template <class Policies>
static size_t _simplify_mesh_with_policies(
    SurfaceMesh &mesh,
    AttachedUvPropertyMap &uv_map,
    const Policies &policies,
    const SimplificationArgs args) {
    typedef typename Policies::Get_cost Cost;
    typedef typename Policies::Get_placement Placement;

    const Cost &cost = policies.get_cost();
    const Placement &placement = policies.get_placement();

    return _simplify_mesh_with_cost_and_placement(mesh, uv_map, cost, placement, args);
}

static size_t _simplify_mesh(
    SurfaceMesh &mesh,
    AttachedUvPropertyMap &uv_map,
    const Algorithm algorithm,
    const SimplificationArgs args) {
    LOG_TRACE("Simplifying mesh (stop ratio={:g}, borders={}, algorithm={})", args.stop_edge_ratio, args.lock_borders ? "Locked" : "Unlocked", algorithm);

    switch (algorithm) {
    case Algorithm::GarlandHeckbert:
        typedef CGAL::Surface_mesh_simplification::GarlandHeckbert_policies<SurfaceMesh, Kernel> GH_policies;
        return _simplify_mesh_with_policies<GH_policies>(mesh, uv_map, GH_policies(mesh), args);
    case Algorithm::LindstromTurk:
        typedef CGAL::Surface_mesh_simplification::LindstromTurk_cost<SurfaceMesh> LT_cost;
        typedef CGAL::Surface_mesh_simplification::LindstromTurk_placement<SurfaceMesh> LT_placement;
        return _simplify_mesh_with_cost_and_placement<LT_cost, LT_placement>(mesh, uv_map, LT_cost(), LT_placement(), args);
    }

    throw std::invalid_argument("invalid algorithm specified");
}

static double measure_max_absolute_error(const SurfaceMesh &original, const SurfaceMesh &simplified, const double bound_on_error = 0.0001) {
    const double error = CGAL::Polygon_mesh_processing::bounded_error_Hausdorff_distance<CGAL::Parallel_if_available_tag, SurfaceMesh, SurfaceMesh>(
        original, simplified, bound_on_error);
    return error + bound_on_error;
}

Result simplify::simplify_mesh(const TerrainMesh &mesh, const StopCondition stop_condition, Options options) {
    // simplification fails with large numerical values so we normalize the values here.
    // TODO: Try to use EPECK instead
    const size_t vertex_count = mesh.positions.size();
    glm::dvec3 average_position(0, 0, 0);
    for (size_t i = 0; i < vertex_count; i++) {
        average_position += mesh.positions[i] / static_cast<double>(vertex_count);
    }

    TerrainMesh normalized_mesh = mesh;
    for (size_t i = 0; i < vertex_count; i++) {
        const glm::vec3 normalized_position = mesh.positions[i] - average_position;
        normalized_mesh.positions[i] = normalized_position;
    }

    SurfaceMesh cgal_mesh = convert::mesh2cgal(normalized_mesh);
    const SurfaceMesh original_mesh(cgal_mesh);

    AttachedUvPropertyMap uv_map = cgal_mesh.add_property_map<VertexDescriptor, Point2>("h:uv").first;
    for (size_t i = 0; i < mesh.uvs.size(); i++) {
        uv_map[CGAL::SM_Vertex_index(i)] = convert::glm2cgal(mesh.uvs[i]);
    }

    double simplification_error = 0;
    // TODO: refactor this
    std::visit(overloaded{
                   [&](EdgeRatio edge_ratio) {
                       // Just simplify until we reach the target primitive count.
                       const SimplificationArgs args{
                           .lock_borders = options.lock_borders,
                           .stop_edge_ratio = edge_ratio.ratio};
                       _simplify_mesh(cgal_mesh, uv_map, options.algorithm, args);
                       simplification_error = measure_max_absolute_error(original_mesh, cgal_mesh);
                   },
                   [&](RelativeError relative_error) {
                       // TODO: implement this
                       throw std::runtime_error("not yet implemented");
                   },
                   [&](AbsoluteError absolute_error) {
                       // TODO: Repeatedly simplify the mesh until we overstep the target error or understep the min stop ratio.
                       // TODO: use something like https://github.com/afabri/cgal/blob/SMS-undo_example-GF/Surface_mesh_simplification/examples/Surface_mesh_simplification/undo_edge_collapse_surface_mesh.cpp

                       const double max_error = absolute_error.error_bound;
                       const double min_stop_ratio = 0;

                       LOG_DEBUG("Simplifying with max error of {:g}", max_error);

                       double last_safe_stop_ratio = 1.0;
                       double compound_stop_ratio = 1.0;
                       // TODO: binary search like strategy
                       while (true) {
                           const SimplificationArgs args{
                               .lock_borders = options.lock_borders,
                               .stop_edge_ratio = 0.9};
                           compound_stop_ratio *= args.stop_edge_ratio;

                           LOG_DEBUG("Trying stop ratio of {:g}", compound_stop_ratio);

                           if (compound_stop_ratio < min_stop_ratio) {
                               break;
                           }

                           const size_t removed_edge_count = _simplify_mesh(cgal_mesh, uv_map, options.algorithm, args);
                           if (removed_edge_count == 0) {
                               LOG_DEBUG("No edges were removed using final stop ratio of {:g}", compound_stop_ratio);
                               break;
                           }

                            // TODO: smarter error bound?
                           const double current_simplification_error = measure_max_absolute_error(original_mesh, cgal_mesh, max_error * 0.1);

                        // TODO: consider accumulated error
                           // TODO: use CGAL::Polygon_mesh_processing::is_Hausdorff_distance_larger() instead
                           LOG_DEBUG("Stop ratio of {:g} achieved error of {:g}", compound_stop_ratio, current_simplification_error);
                           if (current_simplification_error > max_error) {
                               LOG_DEBUG("Max error exceeded, using {:g} as final stop ratio", last_safe_stop_ratio);
                               break;
                           }

                           last_safe_stop_ratio = compound_stop_ratio;
                       }

                       cgal_mesh.clear();
                       cgal_mesh.collect_garbage();
                       cgal_mesh = original_mesh;
                       uv_map = cgal_mesh.add_property_map<VertexDescriptor, Point2>("h:uv").first;
                       for (size_t i = 0; i < mesh.uvs.size(); i++) {
                           uv_map[CGAL::SM_Vertex_index(i)] = convert::glm2cgal(mesh.uvs[i]);
                       }

                       const SimplificationArgs args{
                           .lock_borders = options.lock_borders,
                           .stop_edge_ratio = last_safe_stop_ratio};
                       _simplify_mesh(cgal_mesh, uv_map, options.algorithm, args);

                       simplification_error = measure_max_absolute_error(original_mesh, cgal_mesh, max_error * 0.01);
                   }},
               stop_condition);

    if (!CGAL::Polygon_mesh_processing::experimental::remove_self_intersections(cgal_mesh)) {
        LOG_WARN("Failed to remove self intersections after simplification");
    }

    TerrainMesh simplified_mesh = convert::cgal2mesh(cgal_mesh);
    simplified_mesh.uvs.resize(simplified_mesh.vertex_count());
    for (size_t i = 0; i < CGAL::num_vertices(cgal_mesh); i++) {
        simplified_mesh.uvs[i] = convert::cgal2glm(uv_map[CGAL::SM_Vertex_index(i)]);
    }
    for (size_t i = 0; i < CGAL::num_vertices(cgal_mesh); i++) {
        simplified_mesh.positions[i] += average_position;
    }

    remove_isolated_vertices(simplified_mesh);

    validate_mesh(simplified_mesh);
    validate_mesh(convert::mesh2cgal(simplified_mesh));

    return Result{
        .mesh = simplified_mesh,
        .max_absolute_error = simplification_error};
}

cv::Mat simplify::simplify_texture(const cv::Mat &texture, glm::uvec2 target_resolution) {
    cv::Mat simplified_texture;
    cv::resize(texture, simplified_texture, cv::Size(target_resolution.x, target_resolution.y), cv::INTER_LINEAR);
    return simplified_texture;
}

void simplify::simplify_mesh_texture(TerrainMesh &mesh, glm::uvec2 target_resolution) {
    if (mesh.texture.has_value()) {
        mesh.texture = simplify_texture(mesh.texture.value(), target_resolution);
    }
}
