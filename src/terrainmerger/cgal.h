#ifndef CGAL_H
#define CGAL_H

#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#define DEFINE_KERNEL(K) \
    using Kernel = K; \
    using Point2 = Kernel::Point_2; \
    using Point3 = Kernel::Point_3; \
    using SurfaceMesh = CGAL::Surface_mesh<Point3>; \
    using VertexDescriptor = boost::graph_traits<SurfaceMesh>::vertex_descriptor; \
    using HalfedgeDescriptor = boost::graph_traits<SurfaceMesh>::halfedge_descriptor; \
    using EdgeDescriptor = boost::graph_traits<SurfaceMesh>::edge_descriptor; \
    using FaceDescriptor = boost::graph_traits<SurfaceMesh>::face_descriptor;

namespace kernel {
namespace epick {
    DEFINE_KERNEL(CGAL::Exact_predicates_inexact_constructions_kernel);
}

namespace epeck {
    DEFINE_KERNEL(CGAL::Exact_predicates_exact_constructions_kernel);
}

namespace simple {
    DEFINE_KERNEL(CGAL::Simple_cartesian<double>);
}
}

DEFINE_KERNEL(CGAL::Exact_predicates_inexact_constructions_kernel)

#endif
