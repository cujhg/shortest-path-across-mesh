#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Simple_cartesian.h>

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Surface_mesh.h>

#include <cmath>
#include <cstdlib>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;

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
  cgal_shortest_path(std::vector<std::vector<double>> vertices, std::vector<std::vector<double>> faces)
  {
    int vertices_length = vertices.size();
    int faces_length = faces.size();

    vertex_array vd(vertices_length);
    for (int i = 0; i < vertices_length; i++)
    {
      vd[i] = tmesh.add_vertex(Point_3(vertices[i][0], vertices[i][1], vertices[i][2]));
    }


    for (int i = 0; i < faces_length; i++)
    {
      face_descriptor f = tmesh.add_face(vd[faces[i][0]], vd[faces[i][1]], vd[faces[i][2]]);
    
      if(f == Triangle_mesh::null_face())
      {
        std::cerr<<"The face could not be added because of an orientation error."<<std::endl;
        
      
      }
    }
    
  nan.append(NAN);
} 
  

  
  py::list intersect(std::vector<double> ray_origin, std::vector<double> ray_ending)
  {
    
    Surface_mesh_shortest_path shortest_paths(tmesh);

    const Ray_3 ray(Point_3(ray_origin[0], ray_origin[1], ray_origin[2]), Point_3(ray_ending[0], ray_ending[1], ray_ending[2]));

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
  
  double shortest_distance(std::vector<double> source, std::vector<double> target)
  {
    
    if (std::isnan(source[0]) || std::isnan(source[1]) || std::isnan(source[2]))
    {
    return NAN; 
    }
    
    if (std::isnan(target[0]) || std::isnan(target[1]) || std::isnan(target[2]))
    {
    return NAN; 
    }
    
    Surface_mesh_shortest_path shortest_paths(tmesh);
    
    AABB_tree tree;
    shortest_paths.build_aabb_tree(tree);
    
    Face_location source_loc = shortest_paths.locate<AABB_face_graph_traits>(Point_3(source[0], source[1], source[2]), tree);
    Face_location target_loc = shortest_paths.locate<AABB_face_graph_traits>(Point_3(target[0], target[1], target[2]), tree);

    
    shortest_paths.add_source_point(source_loc.first, source_loc.second);

    Shortest_path_result result = shortest_paths.shortest_distance_to_source_points(target_loc.first, target_loc.second);
    shortest_paths.remove_source_point(result.second);
    
    return result.first;
  }
  
  
  py::array shortest_path(std::vector<double> source, std::vector<double> target)
  {
    
    if (std::isnan(source[0]) || std::isnan(source[1]) || std::isnan(source[2]))
    {
    std::vector<std::vector<double>> return_points(1 , std::vector<double> (3));
    return_points[0] = {NAN, NAN, NAN};
    return py::cast(return_points); 
    }
    
    if (std::isnan(target[0]) || std::isnan(target[1]) || std::isnan(target[2]))
    {
    std::vector<std::vector<double>> return_points(1 , std::vector<double> (3));
    return_points[0] = {NAN, NAN, NAN};
    return py::cast(return_points); 
    }
    
    Surface_mesh_shortest_path
    shortest_paths(tmesh);
    
    AABB_tree tree;
    shortest_paths.build_aabb_tree(tree);
    
    Face_location source_loc = shortest_paths.locate<AABB_face_graph_traits>(Point_3(source[0], source[1], source[2]), tree);
    Face_location target_loc = shortest_paths.locate<AABB_face_graph_traits>(Point_3(target[0], target[1], target[2]), tree);

    shortest_paths.add_source_point(source_loc.first, source_loc.second);

    std::vector<Traits::Point_3> points;
    Shortest_path_result result = shortest_paths.shortest_path_points_to_source_points(target_loc.first, target_loc.second, std::back_inserter(points));
    shortest_paths.remove_source_point(result.second);
    
    std::vector<std::vector<double>> return_points(int(points.size()) , std::vector<double> (3));    
    
    for (int i = 0; i < int(points.size()); i++)
    {
    return_points[i][0] = CGAL::to_double(points[i][0]);
    return_points[i][1] = CGAL::to_double(points[i][1]);
    return_points[i][2] = CGAL::to_double(points[i][2]);
    }
    
    return py::cast(return_points);
  }
  
};


PYBIND11_MODULE(cgal_shortest_path, handle) {
	handle.doc() = "";
	
	py::class_<cgal_shortest_path>(handle, "cgal_shortest_path")
	.def(py::init<std::vector<std::vector<double>>, std::vector<std::vector<double>>>())
	.def("intersect", &cgal_shortest_path::intersect)
	.def("shortest_distance", &cgal_shortest_path::shortest_distance)
	.def("shortest_path", &cgal_shortest_path::shortest_path);
;

}
