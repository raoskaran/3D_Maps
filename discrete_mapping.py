# Code for generating a 3D model out of discrete (separate wall and corner) scene files

import open3d as o3d 
import pyvista as pv
from pyvista import examples
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
add = (wall1+corner12+wall2+corner23+wall3).elevation()
add.texture_map_to_plane(inplace=True)
add.save("map_outputs/discrete_scene.stl")

p = pv.Plotter()

# Controlling the text properties of the scalar bar

sargs = dict(
    title_font_size=30,
    label_font_size=30,
    shadow=True,
    n_labels=3,
    italic=True,
    fmt="%.1f",
    font_family="arial",
)

lighting = False
specular = 1.0
diffuse = 0.5
ambient = 0.5
specular_power = 15

# Adding texture (add texture=tex in the params to enable)

# image_file = examples.mapfile
# tex = pv.read_texture(image_file)
# tex = examples.download_masonry_texture()

p.add_mesh(add, interpolate_before_map=True, show_edges=True, smooth_shading=True, scalar_bar_args=sargs, specular=specular, specular_power=specular_power, diffuse=diffuse, ambient=ambient)
p.enable_eye_dome_lighting()
p.add_floor('z')
p.show()