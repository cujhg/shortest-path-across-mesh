# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.15)

project( cgal_shortest_path)

add_subdirectory(extern/pybind11)

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()



# Creating entries for target: project_point
# ############################
set(CMAKE_SHARED_MODULE_PREFIX "")


pybind11_add_module(cgal_shortest_path cgal_shortest_path.cpp)
add_to_cached_list( CGAL_EXECUTABLE_TARGETS cgal_shortest_path )

# Link the executable to CGAL and third-party libraries
target_link_libraries(cgal_shortest_path PRIVATE CGAL::CGAL)

