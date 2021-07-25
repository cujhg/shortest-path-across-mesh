import trimesh
import cgal_shortest_path
from math import isnan, nan
import numpy as np

mesh = trimesh.load_mesh('data/elephant.stl')
mesh_from_data = cgal_shortest_path.cgal_shortest_path(mesh.vertices, mesh.faces)

start_point = mesh_from_data.intersect([1.0, 1.0, 1.0], [-1.0, -1.0, -1.0])
end_point = mesh_from_data.intersect([1.0, 0.5, 1.0], [-1.0, -0.5, -1.0])

path = mesh_from_data.shortest_path([-0.5, -0.5, -0.25], [0, 1, 0])
distance = mesh_from_data.shortest_distance([-0.5, 0.5, -0.25], [0, 1, 0])

print(distance)
print(path)

if isnan(path[0][0]) == False:
    path_visual = trimesh.load_path(path)
    mesh.visual.face_colors = [255,255,255,255]

    scene = trimesh.Scene([path_visual, mesh])
    scene.show()
