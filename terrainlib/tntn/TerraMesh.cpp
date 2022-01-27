#include "tntn/TerraMesh.h"
#include "tntn/TerraUtils.h"
#include "tntn/logging.h"
#include "tntn/SurfacePoints.h"
#include "tntn/DelaunayTriangle.h"
#include "algorithms/raster_triangle_scanline.h"

#include <glm/glm.hpp>
#include <iostream>
#include <fstream>
#include <array>
#include <unordered_map>
#include <cmath>

namespace tntn {
namespace terra {

void TerraMesh::greedy_insert(double max_error)
{
    m_max_error = max_error;
    m_counter = 0;
    const auto w = m_raster->get_width();
    const auto h = m_raster->get_height();
    TNTN_ASSERT(w > 0);
    TNTN_ASSERT(h > 0);

    TNTN_LOG_DEBUG("starting greedy insertion with raster width: {}, height: {}", w, h);

    // Initialize m_used
    m_used.allocate(w, h);
    m_used.set_all(0);

    // Ensure the four corners are not NAN, otherwise the algorithm can't proceed.
    this->repair_point(0, 0);
    this->repair_point(0, h - 1);
    this->repair_point(w - 1, h - 1);
    this->repair_point(w - 1, 0);

    // Initialize the mesh to two triangles with the height field grid corners as vertices
    TNTN_LOG_DEBUG("initialize the mesh with four corner points");
    this->init_mesh(glm::dvec2(0, 0), glm::dvec2(0, h - 1), glm::dvec2(w - 1, h - 1), glm::dvec2(w - 1, 0));

    m_used.value(0, 0) = 1;
    m_used.value(h - 1, 0) = 1;
    m_used.value(h - 1, w - 1) = 1;
    m_used.value(0, w - 1) = 1;

    // Initialize m_token
    m_token.allocate(w, h);
    m_token.set_all(0);

    // Scan all the triangles and push all candidates into a stack
    // these are actually only the two triangles of the quad.
    dt_ptr t = m_first_face;
    while(t)
    {
        scan_triangle(t);
        t = t->getLink();
    }

    // Iterate until the error threshold is met
    while(!m_candidates.empty())
    {
        Candidate candidate = m_candidates.grab_greatest();

        if(candidate.importance < m_max_error) continue;

        // Skip if the candidate is not the latest
        if(m_token.value(candidate.y, candidate.x) != candidate.token) continue;

        m_used.value(candidate.y, candidate.x) = 1;

        //TNTN_LOG_DEBUG("inserting point: ({}, {}, {})", candidate.x, candidate.y, candidate.z);
        // insert will recursively call scan_triangle through a virtual function call.
        this->insert(glm::dvec2(candidate.x, candidate.y), candidate.triangle);
    }

    TNTN_LOG_DEBUG("finished greedy insertion");
}

void TerraMesh::scan_triangle(dt_ptr t)
{
    Plane z_plane;
    compute_plane(z_plane, t, *m_raster);

    // i didn't write the algorithm, but i think the triangle points should be integer always.
    // maybe that happened during one of my refactorings, e.g., while removing +0.5 in the raster image class.
    // leaving this to be certain / catch errors.
    assert(std::abs(t->point1().x - std::floor(t->point1().x)) < 0.0000000000001);
    assert(std::abs(t->point1().y - std::floor(t->point1().y)) < 0.0000000000001);

    assert(std::abs(t->point2().x - std::floor(t->point2().x)) < 0.0000000000001);
    assert(std::abs(t->point2().y - std::floor(t->point2().y)) < 0.0000000000001);

    assert(std::abs(t->point3().x - std::floor(t->point3().x)) < 0.0000000000001);
    assert(std::abs(t->point3().y - std::floor(t->point3().y)) < 0.0000000000001);

    // oh, the original raster scanline algorithm was buggy, so i replaced it with a unit tested one.

    glm::uvec2 p0 = glm::uvec2(t->point1());
    glm::uvec2 p1 = glm::uvec2(t->point2());
    glm::uvec2 p2 = glm::uvec2(t->point3());


    Candidate candidate = {0, 0, 0.0, -DBL_MAX, m_counter++, t};
    const double no_data_value = m_raster->get_no_data_value();
    const auto fun = [&](const glm::uvec2 coord, double height) {
      m_tested.value(coord.y, coord.x)++;
      if (m_used.value(coord.y, coord.x))
        return;
      if (is_no_data(height, no_data_value))
        return;

      const auto predicted_height = z_plane.eval(coord.x, coord.y);
      const auto error = std::abs(predicted_height - height);
      candidate.consider(coord.x, coord.y, height, error);
    };

    raster::triangle_scanline(*m_raster, p0, p1, p2, fun);

    // We have now found the appropriate candidate point
    m_token.value(candidate.y, candidate.x) = candidate.token;

    // Push the candidate into the stack
    m_candidates.push_back(candidate);
}

std::unique_ptr<Mesh> TerraMesh::convert_to_mesh()
{
    // Find all the vertices
    int w = m_raster->get_width();
    int h = m_raster->get_height();

    std::vector<Vertex> mvertices;

    Raster<int> vertex_id;
    vertex_id.allocate(w, h);
    vertex_id.set_all(0);

    const double no_data_value = m_raster->get_no_data_value();
    int index = 0;
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            if(m_used.value(y, x) == 1)
            {
                const double z = m_raster->value(y, x);
                if(is_no_data(z, no_data_value))
                {
                    continue;
                }

                Vertex v = Vertex({m_raster->col2x(x), m_raster->row2y(y), z});
                mvertices.push_back(v);
                vertex_id.value(y, x) = index;
                index++;
            }
        }
    }

    // Find all the faces
    std::vector<Face> mfaces;
    dt_ptr t = m_first_face;
    while(t)
    {
        Face f;

        glm::dvec2 p1 = t->point1();
        glm::dvec2 p2 = t->point2();
        glm::dvec2 p3 = t->point3();

        if(!ccw(p1, p2, p3))
        {
            f[0] = vertex_id.value((int)p1.y, (int)p1.x);
            f[1] = vertex_id.value((int)p2.y, (int)p2.x);
            f[2] = vertex_id.value((int)p3.y, (int)p3.x);
        }
        else
        {
            f[0] = vertex_id.value((int)p3.y, (int)p3.x);
            f[1] = vertex_id.value((int)p2.y, (int)p2.x);
            f[2] = vertex_id.value((int)p1.y, (int)p1.x);
        }

        mfaces.push_back(f);

        t = t->getLink();
    }

    // now initialise our mesh class with this
    auto mesh = std::make_unique<Mesh>();
    mesh->from_decomposed(std::move(mvertices), std::move(mfaces));
    return mesh;
}

} //namespace terra
} //namespace tntn
