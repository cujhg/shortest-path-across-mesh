#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Simple_cartesian.h>

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Surface_mesh.h>

#include <boost/lexical_cast.hpp>

#include <cmath>
#include <boost/python.hpp>

#include <cstdlib>
#include <fstream>

namespace py = boost::python;

typedef CGAL::Exact_predicates_inexact_constructions_kernel             Kernel;
typedef Kernel::Point_3                                                 Point_3;
typedef Kernel::Ray_3                                                   Ray_3;

typedef CGAL::Surface_mesh<Point_3>                                     Triangle_mesh;

typedef CGAL::Surface_mesh_shortest_path_traits<Kernel, Triangle_mesh>  Traits;
typedef CGAL::Surface_mesh_shortest_path<Traits>                        Surface_mesh_shortest_path;

typedef boost::graph_traits<Triangle_mesh>                              Graph_traits;
typedef Graph_traits::vertex_iterator                                   vertex_iterator;
typedef Graph_traits::face_iterator                                     face_iterator;

typedef typename Surface_mesh_shortest_path::Barycentric_coordinates    Barycentric_coordinates;
typedef typename Surface_mesh_shortest_path::Face_location              Face_location;
typedef typename Surface_mesh_shortest_path::Shortest_path_result       Shortest_path_result;

typedef CGAL::AABB_face_graph_triangle_primitive<Triangle_mesh>         AABB_face_graph_primitive;
typedef CGAL::AABB_traits<Kernel, AABB_face_graph_primitive>            AABB_face_graph_traits;
typedef CGAL::AABB_tree<AABB_face_graph_traits>                         AABB_tree;

typedef Triangle_mesh::Vertex_index                                     vertex_descriptor;
typedef std::vector<vertex_descriptor>                                  vertex_array;
typedef Triangle_mesh::Face_index                                       face_descriptor;

class cgal_shortest_path
{
private:
  Triangle_mesh tmesh;
  py::list nan;
public:
  cgal_shortest_path(py::list vertices, py::list faces)
  {
    int vertices_length = py::len(vertices);
    int faces_length = py::len(faces);

    vertex_array vd(vertices_length);
    for (int i = 0; i < vertices_length; i++)
    {
      double xval = py::extract<double>(vertices[i][0]);
      double yval = py::extract<double>(vertices[i][1]);
      double zval = py::extract<double>(vertices[i][2]);
      vd[i] = tmesh.add_vertex(Point_3(xval, yval, zval));
    }


    for (int i = 0; i < faces_length; i++)
    {
      int xval = py::extract<int>(faces[i][0]);
      int yval = py::extract<int>(faces[i][1]);
      int zval = py::extract<int>(faces[i][2]);
      face_descriptor f = tmesh.add_face(vd[xval], vd[yval], vd[zval]);
    
      if(f == Triangle_mesh::null_face())
      {
        std::cerr<<"The face could not be added because of an orientation error."<<std::endl;
        
      
      }
    }
    
  nan.append(NAN);
  }
  
  cgal_shortest_path(std::string mesh_name)
  {
  std::ifstream input(mesh_name);
  input >> tmesh;
  input.close();
  
  }
  
  py::list intersect(py::list ray_origin, py::list ray_ending)
  {
    
    double xval_o = py::extract<double>(ray_origin[0]);
    double yval_o = py::extract<double>(ray_origin[1]);
    double zval_o = py::extract<double>(ray_origin[2]);
    
    double xval_e = py::extract<double>(ray_ending[0]);
    double yval_e = py::extract<double>(ray_ending[1]);
    double zval_e = py::extract<double>(ray_ending[2]);
    
    Surface_mesh_shortest_path shortest_paths(tmesh);

    const Ray_3 ray(Point_3(xval_o, yval_o, zval_o), Point_3(xval_e, yval_e, zval_e));

    AABB_tree tree;
    shortest_paths.build_aabb_tree(tree);
    Face_location target_loc = shortest_paths.locate<AABB_face_graph_traits>(ray, tree);
    
    Point_3 projection_point;
    if(target_loc.first == boost::graph_traits<Triangle_mesh>::null_face())
    {
    projection_point = Point_3(NAN, NAN, NAN);
    }
    
    else
    {
    projection_point = shortest_paths.point(target_loc.first, target_loc.second);
    }

    py::list return_values;
    return_values.append(CGAL::to_double(projection_point[0]));
    return_values.append(CGAL::to_double(projection_point[1]));
    return_values.append(CGAL::to_double(projection_point[2]));

    return return_values;
  }
  
  double shortest_distance(py::list source, py::list target)
  {
    
    double source_cart[3];
    source_cart[0] = py::extract<double>(source[0]);
    source_cart[1] = py::extract<double>(source[1]);
    source_cart[2] = py::extract<double>(source[2]);
    
    if (std::isnan(source_cart[0]) || std::isnan(source_cart[1]) || std::isnan(source_cart[2]))
    {
    return NAN; 
    }
    
    double target_cart[3];
    target_cart[0] = py::extract<double>(target[0]);
    target_cart[1] = py::extract<double>(target[1]);
    target_cart[2] = py::extract<double>(target[2]);
    
    if (std::isnan(target_cart[0]) || std::isnan(target_cart[1]) || std::isnan(target_cart[2]))
    {
    return NAN; 
    }
    
    Surface_mesh_shortest_path shortest_paths(tmesh);
    
    AABB_tree tree;
    shortest_paths.build_aabb_tree(tree);
    
    Face_location source_loc = shortest_paths.locate<AABB_face_graph_traits>(Point_3(source_cart[0], source_cart[1], source_cart[2]), tree);
    Face_location target_loc = shortest_paths.locate<AABB_face_graph_traits>(Point_3(target_cart[0], target_cart[1], target_cart[2]), tree);

    
    shortest_paths.add_source_point(source_loc.first, source_loc.second);

    Shortest_path_result result = shortest_paths.shortest_distance_to_source_points(target_loc.first, target_loc.second);
    shortest_paths.remove_source_point(result.second);
    
    return result.first;
  }
  
  py::list shortest_path(py::list source, py::list target)
  {
    
    py::list return_points;
    
    double source_cart[3];
    source_cart[0] = py::extract<double>(source[0]);
    source_cart[1] = py::extract<double>(source[1]);
    source_cart[2] = py::extract<double>(source[2]);
    
    if (std::isnan(source_cart[0]) || std::isnan(source_cart[1]) || std::isnan(source_cart[2]))
    {
    return_points.append(nan);
    return return_points; 
    }
    
    double target_cart[3];
    target_cart[0] = py::extract<double>(target[0]);
    target_cart[1] = py::extract<double>(target[1]);
    target_cart[2] = py::extract<double>(target[2]);
    
    if (std::isnan(target_cart[0]) || std::isnan(target_cart[1]) || std::isnan(target_cart[2]))
    {
    return_points.append(nan);
    return return_points; 
    }
    
    Surface_mesh_shortest_path
    shortest_paths(tmesh);
    
    AABB_tree tree;
    shortest_paths.build_aabb_tree(tree);
    
    Face_location source_loc = shortest_paths.locate<AABB_face_graph_traits>(Point_3(source_cart[0], source_cart[1], source_cart[2]), tree);
    Face_location target_loc = shortest_paths.locate<AABB_face_graph_traits>(Point_3(target_cart[0], target_cart[1], target_cart[2]), tree);

    shortest_paths.add_source_point(source_loc.first, source_loc.second);

    std::vector<Traits::Point_3> points;
    Shortest_path_result result = shortest_paths.shortest_path_points_to_source_points(target_loc.first, target_loc.second, std::back_inserter(points));
    shortest_paths.remove_source_point(result.second);
    

    
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        py::list return_point ={};
        return_point.append(CGAL::to_double(points[i][0]));
        return_point.append(CGAL::to_double(points[i][1]));
        return_point.append(CGAL::to_double(points[i][2]));
        return_points.append(return_point);
    }
    return return_points;
  }
};


using namespace boost::python;

BOOST_PYTHON_MODULE(cgal_shortest_path)
{
    class_<cgal_shortest_path>("cgal_shortest_path", init<py::list, py::list>())
    .def(init<std::string>())
    .def("intersect", &cgal_shortest_path::intersect)
    .def("shortest_distance", &cgal_shortest_path::shortest_distance)
    .def("shortest_path", &cgal_shortest_path::shortest_path)
    ;
}
