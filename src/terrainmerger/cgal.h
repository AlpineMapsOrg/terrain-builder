#ifndef CGAL_H
#define CGAL_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point2;
typedef Kernel::Point_3 Point3;
typedef CGAL::Surface_mesh<Point3> SurfaceMesh;

typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor VertexDescriptor;
typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor HalfedgeDescriptor;
typedef boost::graph_traits<SurfaceMesh>::edge_descriptor EdgeDescriptor;
typedef boost::graph_traits<SurfaceMesh>::face_descriptor FaceDescriptor;

#endif
