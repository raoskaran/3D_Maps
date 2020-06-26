import open3d as o3d 
import pyvista as pv
from pyvista import examples
import numpy as np, pandas as pd

def gen_surface(pcd):
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.095)
    pcd, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=1.0)
    pcd.estimate_normals()
    points = np.asarray(pcd.points)
    points = points[~np.all(points == 0, axis=1)]
    pointcloud = pv.PolyData(points)
    pointcloud['order'] = np.linspace(0,1,pointcloud.n_points)
    surf = pointcloud.delaunay_2d(tol=1e-05, alpha=1.0, offset=2.0, progress_bar=False)
    surf.compute_normals(inplace=True)
    surf = surf.smooth(n_iter = 200)
    return surf


pcd = o3d.io.read_point_cloud("data/scenes/continous_final.xyz")

scene = gen_surface(pcd)
scene = scene.elevation()
scene.texture_map_to_plane(inplace=True)
scene.save("map_outputs/continuous_scene.stl")

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

p.add_mesh(scene, interpolate_before_map=True, show_edges=True, smooth_shading=True, scalar_bar_args=sargs, specular=specular, specular_power=specular_power, diffuse=diffuse, ambient=ambient)
p.enable_eye_dome_lighting()
p.add_floor('z')
p.show()