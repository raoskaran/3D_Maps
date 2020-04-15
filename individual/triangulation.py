import pyvista as pv
import pandas as pd
import numpy as np
import open3d as o3d

pv.set_plot_theme('document')

# df = pd.read_csv('data/chair.csv')
# points = df[['x', 'y', 'z']].values
pcd = o3d.io.read_point_cloud("../data/chair.xyz")
voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
pcd, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
points = np.asarray(pcd.points)
points = points[~np.all(points == 0, axis=1)]
cloud = pv.PolyData(points)
cloud['order'] = np.linspace(0,1,cloud.n_points)

surf = cloud.delaunay_2d(alpha=1.0)
surf = surf.smooth(n_iter = 500)
surf.plot(show_edges=True, show_grid=False, cpos="xy")
