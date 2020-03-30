import numpy as np
import pandas as pd
import binvox_rw
from pyntcloud import PyntCloud
import mcubes
# import binvox_rw

cloud = PyntCloud.from_file("points.txt", sep=",")


voxelgrid_id = cloud.add_structure("voxelgrid", n_x=32, n_y=32, n_z=32)

voxelgrid = cloud.structures[voxelgrid_id]

x_cords = voxelgrid.voxel_x
y_cords = voxelgrid.voxel_y
z_cords = voxelgrid.voxel_z

voxel = np.zeros((32, 32, 32))

for x, y, z in zip(x_cords, y_cords, z_cords):
    voxel[x][y][z] = 1

# smooth = mcubes.smooth(voxel)
vertices, triangles = mcubes.marching_cubes(voxel, 0)

mcubes.export_mesh(vertices, triangles, "scene.dae", "MySphere")