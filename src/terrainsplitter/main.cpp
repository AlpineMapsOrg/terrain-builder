#include "cgal.h"
#include "cli.h"
#include "convert.h"
#include "log.h"
#include "mesh/io.h"
#include "validate.h"

#include <CGAL/Surface_mesh_approximation/approximate_triangle_mesh.h>
#include <metis.h>

TerrainMesh load_mesh_from_path(const std::filesystem::path &path) {
    auto result = io::load_mesh_from_path(path);
    if (!result.has_value()) {
        const io::LoadMeshError error = result.error();
        LOG_ERROR("Failed to load mesh from {}: {}", path.string(), error.description());
        exit(EXIT_FAILURE);
    }

    const TerrainMesh mesh = result.value();
    validate_mesh(mesh);
    validate_mesh(convert::mesh2cgal(mesh));

    return mesh;
}

void partition_mesh2(const TerrainMesh &mesh, const std::span<TerrainMesh> cluster_meshes) {
    SurfaceMesh cgal_mesh = convert::mesh2cgal(mesh);
    typedef SurfaceMesh::Property_map<FaceDescriptor, std::size_t> FaceProxyPropertyMap;
    FaceProxyPropertyMap cluster_map = cgal_mesh.add_property_map<FaceDescriptor, std::size_t>("f:proxy_id", 0).first;
    CGAL::Surface_mesh_approximation::approximate_triangle_mesh(cgal_mesh,
                                                                CGAL::parameters::verbose_level(CGAL::Surface_mesh_approximation::MAIN_STEPS)
                                                                    .max_number_of_proxies(cluster_meshes.size())
                                                                    .min_error_drop(1)
                                                                    .face_proxy_map(cluster_map));

    std::array<std::vector<size_t>, 4> index_mapping;
    const size_t invalid_index = mesh.vertex_count();
    for (std::vector<size_t> &v : index_mapping) {
        v = {};
        v.resize(mesh.vertex_count());
        std::fill(v.begin(), v.end(), invalid_index);
    }

    for (TerrainMesh &cluster_mesh : cluster_meshes) {
        cluster_mesh = {};
        cluster_mesh.positions.reserve(mesh.vertex_count() / cluster_meshes.size() * 2);
        cluster_mesh.uvs.reserve(mesh.vertex_count() / cluster_meshes.size() * 2);
        cluster_mesh.triangles.reserve(mesh.face_count() / cluster_meshes.size() * 2);
    }

    for (const FaceDescriptor f : CGAL::faces(cgal_mesh)) {
        const size_t cluster_id = cluster_map[f];
        TerrainMesh &cluster_mesh = cluster_meshes[cluster_id];

        std::array<CGAL::SM_Vertex_index, 3> triangle;
        std::array<glm::dvec3, 3> positions;
        unsigned int i = 0;
        for (const CGAL::SM_Vertex_index vertex_index : CGAL::vertices_around_face(cgal_mesh.halfedge(f), cgal_mesh)) {
            triangle[i] = vertex_index;
            positions[i] = convert::cgal2glm(cgal_mesh.point(vertex_index));
            i++;
        }

        glm::uvec3 new_triangle;
        for (size_t k = 0; k < static_cast<size_t>(triangle.size()); k++) {
            const size_t vertex_index = triangle[k];

            size_t &mapped_index = index_mapping[cluster_id][vertex_index];
            if (mapped_index == invalid_index) {
                mapped_index = cluster_mesh.positions.size();
                cluster_mesh.positions.push_back(positions[k]);
            }

            new_triangle[k] = mapped_index;
        }

        cluster_mesh.triangles.push_back(new_triangle);
    }
}

void partition_mesh(const TerrainMesh &mesh, const std::span<TerrainMesh> cluster_meshes) {
    // Convert mesh data to METIS-compatible format
    idx_t ne = static_cast<idx_t>(mesh.triangles.size()); // Number of elements (triangles)
    idx_t nn = static_cast<idx_t>(mesh.positions.size()); // Number of nodes

    // METIS requires arrays for eptr and eind, which represent the mesh as described in Section 5.6
    std::vector<idx_t> eptr(ne + 1); // Element pointer array
    std::vector<idx_t> eind(ne * 3); // Element index array

    // Initialize eptr and eind based on the triangle data<
    eptr[0] = 0;
    for (idx_t i = 0; i < ne; ++i) {
        eptr[i + 1] = eptr[i] + 3; // Assuming each triangle has 3 vertices
        eind[i * 3] = mesh.triangles[i].x;
        eind[i * 3 + 1] = mesh.triangles[i].y;
        eind[i * 3 + 2] = mesh.triangles[i].z;
    }

    // Convert node positions to METIS-compatible format
    std::vector<idx_t> vwgt(nn, 1.0);  // Node weights (assuming equal weights for all nodes)
    std::vector<idx_t> vsize(nn, 1.0); // Node sizes (assuming equal sizes for all nodes)

    // Set the number of parts and other optional parameters
    idx_t nparts = cluster_meshes.size();                                  // Number of parts to partition the mesh
    std::vector<idx_t> options(METIS_NOPTIONS, 0);                         // Initialize options to default values

    // Allocate memory for output vectors
    std::vector<idx_t> epart(ne);
    std::vector<idx_t> npart(nn);

    // Call the METIS_PartMeshNodal function
    idx_t objval;
    // TODO: Try METIS_PartGraphKway as it supports edges weights which could be set to their lengths
    auto result = METIS_PartMeshNodal(&ne, &nn, eptr.data(), eind.data(), nullptr, nullptr,
                        &nparts, nullptr, nullptr, &objval, epart.data(), npart.data());
    assert(METIS_OK == result);

    std::vector<std::vector<size_t>> mapping;
    mapping.resize(nparts);
    idx_t illegal = nn + 1;
    for (std::vector<size_t> &m : mapping) {
        m.resize(nn);
        std::fill(m.begin(), m.end(), illegal);
    }

    // Create vectors to store the partitioned mesh data
    for (TerrainMesh &cluster_mesh : cluster_meshes) {
        cluster_mesh.positions.reserve(nn);
        cluster_mesh.triangles.reserve(ne);
    }

    for (idx_t i = 0; i < nn; ++i) {
        idx_t partId = npart[i]; // Partition ID for the current node

        // Add the current position to the corresponding partition
        mapping[partId][i] = cluster_meshes[partId].positions.size();
        cluster_meshes[partId].positions.push_back(mesh.positions[i]);
    }

    // Populate the partitioned mesh data based on the partition vectors
    for (idx_t i = 0; i < ne; ++i) {
        idx_t partId = epart[i]; // Partition ID for the current element

        // Add the current triangle to the corresponding partition
        glm::uvec3 triangle;
        for (size_t k = 0; k < static_cast<size_t>(triangle.length()); k++) {
            const idx_t source = eind[i * 3 + k];
            size_t& mapped = mapping[partId][source];
            if (mapped == illegal) {
                mapped = cluster_meshes[partId].positions.size();
                cluster_meshes[partId].positions.push_back(mesh.positions[source]);
            }
            triangle[k] = mapped;
        }
        cluster_meshes[partId].triangles.push_back(triangle);
    }
}

template <size_t N>
std::array<TerrainMesh, N> partition_mesh(const TerrainMesh &mesh) {
    std::array<TerrainMesh, N> cluster_meshes;
    partition_mesh(mesh, cluster_meshes);
    return cluster_meshes;
}

std::vector<TerrainMesh> partition_mesh(const TerrainMesh &mesh, size_t num_partitions) {
    std::vector<TerrainMesh> cluster_meshes;
    cluster_meshes.resize(num_partitions);
    partition_mesh(mesh, cluster_meshes);
    return cluster_meshes;
}

void run(const cli::Args &args) {
    LOG_INFO("Loading meshes...");
    const TerrainMesh mesh = load_mesh_from_path(args.input_path);

    const auto clusters = partition_mesh<4>(mesh);

    size_t i = 1;
    for (const TerrainMesh &cluster_mesh : clusters) {
        LOG_INFO("{}", i);
        const auto path = std::filesystem::path(args.input_path).replace_extension(fmt::format(".cluster.{}.glb", i));
        LOG_DEBUG("{}", path.string());
        io::save_mesh_to_path(path, cluster_mesh);
        i++;
    }

    LOG_INFO("Done");
}

int main(int argc, char **argv) {
    std::vector<char *> args2 = {"terrainsplitter", "--input", "./vienna_hierarchy_new5/17/45448/71496.tile", "--output", "ignored"};
    argv = args2.data();
    argc = args2.size();

    const cli::Args args = cli::parse(argc, argv);
    Log::init(args.log_level);

    const std::string arg_str = std::accumulate(argv, argv + argc, std::string(),
                                                [](const std::string &acc, const char *arg) {
                                                    return acc + (acc.empty() ? "" : " ") + arg;
                                                });
    LOG_DEBUG("Running with: {}", arg_str);

    run(args);
}
