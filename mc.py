import numpy as np
import mcubes
from pprint import pprint

X, Y, Z = np.mgrid[:30, :30, :30]

u = (X-15)**2 + (Y-15)**2 + (Z-15)**2 - 8**2

vertices, triangles = mcubes.marching_cubes(u, 0)

mcubes.export_mesh(vertices, triangles, "sphere.dae", "MySphere")