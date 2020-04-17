# Code for applying Delaunay3d on the result of Ball Pivoting (Use tools/vtk2stl.py to convert)

import open3d as o3d 
import pyvista as pv
import numpy as np, pandas as pd
import trimesh

pcd = o3d.io.read_point_cloud("data/scenes/continous_1.xyz")
distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 1.5 * avg_dist
voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
pcd, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
pcd.estimate_normals()
points = np.asarray(pcd.points)
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
           pcd,
           o3d.utility.DoubleVector([radius, radius * 2]))

mesh.compute_vertex_normals()

trimesh = trimesh.Trimesh(np.asarray(mesh.vertices), np.asarray(mesh.triangles), vertex_normals=np.asarray(mesh.vertex_normals))

trimesh.show()

o3d.io.write_triangle_mesh("map_outputs/output.ply", mesh)

mesh = pv.read("map_outputs/output.ply")
mesh.compute_normals()
mesh = mesh.delaunay_3d(alpha=0.5)
mesh.plot(eye_dome_lighting=True, show_edges=True, show_grid=False)
mesh.save("map_outputs/ball_delaunay.vtk")