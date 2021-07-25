# shortest-path-across-mesh
A python module which offers the functionality to compute the shortest path between to points across an arbitrary 3d triangle-mesh.
Tested on Linux.

## Dependencies
1. pybind
2. cgal 5.2
3. cmake 3.16.3
4. trimesh (only for example.py)
The stated versions are the ones with which the module was developed. Other may work as well.

## Installation

1. cd ~/path/to/shortest-path-accross-mesh
2. cmake -DCMAKE_BUILD_TYPE=Release .
3. make
This creates a .so file which can be importet into python.

## Usage
See example.py
