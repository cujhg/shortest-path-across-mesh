# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.15)

project( cgal_shortest_path)


# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

# include for local directory

find_package(PythonLibs 3.8 REQUIRED)
find_package(Boost REQUIRED python38 numpy)
# include for local package


# Creating entries for target: project_point
# ############################
set(CMAKE_SHARED_MODULE_PREFIX "")
#add_executable( project_point  project_point.cpp )
add_library(cgal_shortest_path MODULE cgal_shortest_path.cpp)

add_to_cached_list( CGAL_EXECUTABLE_TARGETS cgal_shortest_path )

# Link the executable to CGAL and third-party libraries
target_link_libraries(cgal_shortest_path PRIVATE CGAL::CGAL ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(cgal_shortest_path PRIVATE ${PYTHON_INCLUDE_DIRS})
