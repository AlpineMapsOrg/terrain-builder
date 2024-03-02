#ifndef CGAL_H
#define CGAL_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
// typedef CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt Kernel;
typedef Kernel::Point_2 Point2;
typedef Kernel::Point_3 Point3;
typedef CGAL::Surface_mesh<Point3> SurfaceMesh;

typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor VertexDescriptor;
typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor HalfedgeDescriptor;
typedef boost::graph_traits<SurfaceMesh>::edge_descriptor EdgeDescriptor;
typedef boost::graph_traits<SurfaceMesh>::face_descriptor FaceDescriptor;

typedef CGAL::Simple_cartesian<double> Kernel_;
typedef Kernel_::Point_2 Point2_;
typedef Kernel_::Point_3 Point3_;
typedef CGAL::Surface_mesh<Point3_> SurfaceMesh_;

typedef boost::graph_traits<SurfaceMesh_>::vertex_descriptor VertexDescriptor_;
typedef boost::graph_traits<SurfaceMesh_>::halfedge_descriptor HalfedgeDescriptor_;
typedef boost::graph_traits<SurfaceMesh_>::edge_descriptor EdgeDescriptor_;
typedef boost::graph_traits<SurfaceMesh_>::face_descriptor FaceDescriptor_;

#endif
