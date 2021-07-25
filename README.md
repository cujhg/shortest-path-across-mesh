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
```console
cd ~/path/to/shortest-path-accross-mesh
cmake -DCMAKE_BUILD_TYPE=Release .
make
```
This creates a .so file which can be importet into python.

## Methods
1. intersect: Find intersection point between the mesh and a line defined by two points
2. shortest_distance: Find the shortest distance between two abitrary points lying on a mesh
3. shortest_path: Find the shortest path between two arbitrary points lying on a mesh

## Usage
See example.py
