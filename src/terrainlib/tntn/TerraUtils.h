#pragma once

#include "DelaunayMesh.h"
#include "DelaunayTriangle.h"
#include "Raster.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <vector>

namespace tntn {
namespace terra {

    struct Candidate {
        unsigned x = 0;
        unsigned y = 0;
        double z = 0.0;
        double importance = -DBL_MAX;
        int token = 0;
        dt_ptr triangle;

        void consider(unsigned sx, unsigned sy, double sz, double imp) noexcept
        {
            if (imp > importance) {
                x = sx;
                y = sy;
                z = sz;
                importance = imp;
            }
        }

        bool operator>(const Candidate& c) const noexcept { return (importance > c.importance); }
        bool operator<(const Candidate& c) const noexcept { return (importance < c.importance); }
    };

    class CandidateList {
    public:
        void push_back(const Candidate& candidate) { m_candidates.push(candidate); }

        size_t size() const noexcept { return m_candidates.size(); }
        bool empty() const noexcept { return m_candidates.empty(); }

        // find greatest element, remove from candidate list and return
        Candidate grab_greatest()
        {
            if (m_candidates.empty()) {
                return Candidate();
            }

            Candidate candidate = m_candidates.top();
            m_candidates.pop();
            return candidate;
        }

    private:
        std::priority_queue<Candidate> m_candidates;
    };

    inline void order_triangle_points(std::array<Point2D, 3>& p) noexcept
    {
        // bubble sort
        if (p[0].y > p[1].y) {
            std::swap(p[0], p[1]);
        }
        if (p[1].y > p[2].y) {
            std::swap(p[1], p[2]);
        }
        if (p[0].y > p[1].y) {
            std::swap(p[0], p[1]);
        }
    }

    inline bool is_no_data(const double value, const double no_data_value) noexcept
    {
        return std::isnan(value) || value == no_data_value;
    }

    inline void compute_plane(Plane& plane, dt_ptr t, const RasterDouble& raster)
    {
        const glm::dvec2& p1 = t->point1();
        const glm::dvec2& p2 = t->point2();
        const glm::dvec2& p3 = t->point3();

        const glm::dvec3 v1(p1, raster.value(unsigned(p1.y), unsigned(p1.x)));
        const glm::dvec3 v2(p2, raster.value(unsigned(p2.y), unsigned(p2.x)));
        const glm::dvec3 v3(p3, raster.value(unsigned(p3.y), unsigned(p3.x)));

        plane.init(v1, v2, v3);
    }

    // abstract base class for Terra and Zemlya
    class TerraBaseMesh : protected DelaunayMesh {
    public:
        TerraBaseMesh()
            : m_raster(std::make_unique<RasterDouble>())
        {
        }

        void load_raster(std::unique_ptr<RasterDouble> raster);

    protected:
        std::unique_ptr<RasterDouble> m_raster;

        void repair_point(unsigned px, unsigned py);
    };

} // namespace terra
} // namespace tntn
