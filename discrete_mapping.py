import open3d as o3d 
import pyvista as pv
import numpy as np, pandas as pd

def gen_surface(pcd):
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    pcd, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
    pcd.estimate_normals()
    points = np.asarray(pcd.points)
    points = points[~np.all(points == 0, axis=1)]
    pointcloud = pv.PolyData(points)
    pointcloud['order'] = np.linspace(0,1,pointcloud.n_points)
    surf = pointcloud.delaunay_2d(alpha=2.0)
    surf.compute_normals()
    surf = surf.smooth(n_iter = 200)
    return surf

pcd1 = o3d.io.read_point_cloud("data/discrete map/wall_1.xyz")
pcd2 = o3d.io.read_point_cloud("data/discrete map/wall_2.xyz")
pcd3 = o3d.io.read_point_cloud("data/discrete map/wall_3.xyz")
pcd4 = o3d.io.read_point_cloud("data/discrete map/corner_12.xyz")
pcd5 = o3d.io.read_point_cloud("data/discrete map/corner_23.xyz")

pv.set_plot_theme('document')

wall1 = gen_surface(pcd1) 
wall1.rotate_z(90)
wall2 = gen_surface(pcd2)
# wall2.rotate_z(270)
wall3 = gen_surface(pcd3)
# wall3.rotate_z(90)
wall3.rotate_z(270)
corner12 = gen_surface(pcd4)
corner12.rotate_z(45)
corner23 = gen_surface(pcd5)
corner23.rotate_z(-45)
add = wall1+corner12+wall2+corner23+wall3
add.plot(eye_dome_lighting=True, show_edges=True, show_grid=False)
add.save("map_outputs/discrete_scene.stl")