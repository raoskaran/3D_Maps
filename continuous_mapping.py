# Code for generating a 3D model out of continuous scene files

import open3d as o3d 
import pyvista as pv
import numpy as np, pandas as pd

def gen_surface(pcd):
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.095)
    pcd, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=1.0)
    pcd.estimate_normals()
    points = np.asarray(pcd.points)
    points = points[~np.all(points == 0, axis=1)]
    pointcloud = pv.PolyData(points)
    pointcloud['order'] = np.linspace(0,1,pointcloud.n_points)
    surf = pointcloud.delaunay_2d(tol=1e-05, alpha=2.0, offset=2.0, progress_bar=False)
    surf.compute_normals()
    surf = surf.smooth(n_iter = 200)
    return surf

pcds = []
surfs = []

for i in range(0,3):
    path = "data/continuous map/file_%s.xyz"
    pcd = o3d.io.read_point_cloud(path%i)
    surf = gen_surface(pcd)
    surfs.append(surf)


add = surfs[0]+surfs[1]+surfs[2]
add.plot(eye_dome_lighting=True, show_edges=True, show_grid=False)
add.save("map_outputs/continuous_scene.stl")