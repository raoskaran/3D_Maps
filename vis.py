# VISUALIZATION TECHNIQUES FOR XYZ POINTCLOUDS

# Uncomment section as per requirement


import open3d as o3d
import trimesh, pandas as pd
import numpy as np
import mcubes
from pyntcloud import PyntCloud
from pprint import pprint


pcd = o3d.io.read_point_cloud("chair.xyz")
pcd.estimate_normals()

# estimate radius for rolling ball
distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 1.5 * avg_dist   

# Ball pivoting

# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
#            pcd,
#            o3d.utility.DoubleVector([radius, radius * 2]))

# trimesh = trimesh.Trimesh(np.asarray(mesh.vertices), np.asarray(mesh.triangles), vertex_normals=np.asarray(mesh.vertex_normals))

# trimesh.show()

# Tetrahederal/Delaunay

mesh = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)

o3d.visualization.draw_geometries([mesh[0]]) # Visualization for tetrahedral

# Poisson reconstruction

# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd)

# trimesh = trimesh.Trimesh(np.asarray(mesh[0].vertices), np.asarray(mesh[0].triangles), vertex_normals=np.asarray(mesh[0].vertex_normals))

# trimesh.show()

# Marching cubes

# cloud = PyntCloud.from_file("chair.xyz", sep=" ")

# voxelgrid_id = cloud.add_structure("voxelgrid", n_x=32, n_y=32, n_z=32)

# voxelgrid = cloud.structures[voxelgrid_id]

# x_cords = voxelgrid.voxel_x
# y_cords = voxelgrid.voxel_y
# z_cords = voxelgrid.voxel_z

# voxel = np.zeros((32, 32, 32))

# for x, y, z in zip(x_cords, y_cords, z_cords):
#     voxel[x][y][z] = 1

# # smooth = mcubes.smooth(voxel)
# vertices, triangles = mcubes.marching_cubes(voxel, 0)

# mcubes.export_mesh(vertices, triangles, "scene.dae", "MyScene")

# Write mesh to file

# o3d.io.write_triangle_mesh("output.ply", mesh) 