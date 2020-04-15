import numpy as np, pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pyntcloud import PyntCloud
from skimage import measure
from skimage.draw import ellipsoid


# Generate a level set about zero of two identical ellipsoids in 3D
# ellip_base = ellipsoid(6, 10, 16, levelset=True)
# ellip_double = np.concatenate((ellip_base[:-1, ...],
#                                ellip_base[2:, ...]), axis=0)

cloud = PyntCloud.from_file("data/points.xyz", sep=" ")
# k_neighbors = cloud.get_neighbors(k=10)
# cloud.add_scalar_field("normals", k_neighbors=k_neighbors)

voxelgrid_id = cloud.add_structure("voxelgrid", n_x=32, n_y=32, n_z=32)

voxelgrid = cloud.structures[voxelgrid_id]

x_cords = voxelgrid.voxel_x
y_cords = voxelgrid.voxel_y
z_cords = voxelgrid.voxel_z

voxel = np.zeros((32, 32, 32))

for x, y, z in zip(x_cords, y_cords, z_cords):
    voxel[x][y][z] = 1
# Use marching cubes to obtain the surface mesh of these ellipsoids
verts, faces, normals, values = measure.marching_cubes_lewiner(voxel, 0)

# Display resulting triangular mesh using Matplotlib. This can also be done
# with mayavi (see skimage.measure.marching_cubes_lewiner docstring).
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

# Fancy indexing: `verts[faces]` to generate a collection of triangles
mesh = Poly3DCollection(verts[faces])
mesh.set_edgecolor('k')
ax.add_collection3d(mesh)

ax.set_xlabel("x-axis: a = 6 per ellipsoid")
ax.set_ylabel("y-axis: b = 10")
ax.set_zlabel("z-axis: c = 16")

ax.set_xlim(0, 24)  # a = 6 (times two for 2nd ellipsoid)
ax.set_ylim(0, 20)  # b = 10
ax.set_zlim(0, 32)  # c = 16

plt.tight_layout()
plt.show()