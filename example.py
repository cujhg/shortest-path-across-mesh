import trimesh
import cgal_shortest_path
from math import isnan, nan


mesh = trimesh.load_mesh('data/elephant.stl')
vertices = mesh.vertices.tolist()
faces = mesh.faces.tolist()

#mesh_from_data = cgal_shortest_path.cgal_shortest_path(vertices, faces)
mesh_from_data = cgal_shortest_path.cgal_shortest_path('data/elephant.off')

point_1 = mesh_from_data.intersect([1.0, 1.0, 1.0], [-1.0, -1.0, -1.0])
point_2 = mesh_from_data.intersect([1.0, 0.5, 1.0], [-1.0, -0.5, -1.0])

path = mesh_from_data.shortest_path([-0.5, -0.5, -0.25], [0, 1, 0])
distance = mesh_from_data.shortest_distance([1, 0, 0], [0, 1, 0])

print(distance)

if isnan(path[0][0]) == False:
    path_visual = trimesh.load_path(path)
    mesh.visual.face_colors = [255,255,255,255]

    scene = trimesh.Scene([path_visual, mesh])
    scene.show()
