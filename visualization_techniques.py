#----------------------------------VISUALIZATION TECHNIQUES FOR XYZ POINTCLOUDS----------------------------------

#----------------------------------Uncomment section as per requirement------------------------------------------

import open3d as o3d
import trimesh, pandas as pd
import numpy as np
import mcubes
import pyvista as pv
import matplotlib.pyplot as plt
from pyntcloud import PyntCloud
from pprint import pprint
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from skimage import measure

#----------------------------------------------Helper functions----------------------------------------------

def draw_geometries_with_back_face(geometries):

    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window()
    render_option = visualizer.get_render_option()
    render_option.mesh_show_back_face = True
    for geometry in geometries:
        visualizer.add_geometry(geometry)
    visualizer.run()
    visualizer.destroy_window()

def generate_voxel(cloud):

    voxelgrid_id = cloud.add_structure("voxelgrid", n_x=32, n_y=32, n_z=32)

    voxelgrid = cloud.structures[voxelgrid_id]

    x_cords = voxelgrid.voxel_x
    y_cords = voxelgrid.voxel_y
    z_cords = voxelgrid.voxel_z

    voxel = np.zeros((32, 32, 32))

    for x, y, z in zip(x_cords, y_cords, z_cords):
        voxel[x][y][z] = 1
    
    return voxel

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_down_sample(ind)
    outlier_cloud = cloud.select_down_sample(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


#----------------------------------------------Preprocessing----------------------------------------------

pcd = o3d.io.read_point_cloud("data/continous_1.xyz")
cloud = PyntCloud.from_file("data/scene.xyz", sep=" ")

# estimate radius for rolling ball
distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 1.5 * avg_dist  

# remove outliers
voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
pcd, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
pcd.estimate_normals()
points = np.asarray(pcd.points)
# display_inlier_outlier(voxel_down_pcd, ind)
# o3d.visualization.draw_geometries([pcd])

#----------------------------------------------Ball pivoting----------------------------------------------

mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
           pcd,
           o3d.utility.DoubleVector([radius, radius * 2]))

mesh.compute_vertex_normals()

trimesh = trimesh.Trimesh(np.asarray(mesh.vertices), np.asarray(mesh.triangles), vertex_normals=np.asarray(mesh.vertex_normals))

trimesh.show()


#----------------------------------------------Tetrahederal-----------------------------------------------

# mesh = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)[0]

# o3d.visualization.draw_geometries([mesh]) # Visualization for tetrahedral

#----------------------------------------------Alpha---------------------------------------------------------

# o3d.utility.set_verbosity_level(o3d.utility.Debug)
# o3d.visualization.draw_geometries([pcd])
# print("compute tetra mesh only once")
# tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
# print("done with tetra mesh")
# for alpha in np.logspace(np.log10(0.5), np.log10(0.01), num=4):
#     print("alpha={}".format(alpha))
#     mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
#         pcd, alpha, tetra_mesh, pt_map)
#     mesh.compute_vertex_normals()
#     draw_geometries_with_back_face([mesh])

#----------------------------------------------Delaunay Triangulation---------------------------------------

# pv.set_plot_theme('document')

# points = points[~np.all(points == 0, axis=1)]
# pointcloud = pv.PolyData(points)
# pointcloud['order'] = np.linspace(0,1,pointcloud.n_points)

# surf = pointcloud.delaunay_2d(alpha=1.0)
# surf = surf.smooth(n_iter = 200)
# surf.plot(eye_dome_lighting=True, show_edges=True, show_grid=False, cpos="xy")
# surf.save("outputs/scene1.stl")

#----------------------------------------------Poisson reconstruction---------------------------------------

# o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

# # o3d.visualization.draw_geometries([pcd])

# print('run Poisson surface reconstruction')
# mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#     pcd, depth=8)
# print(mesh)
# # o3d.visualization.draw_geometries([mesh])

# print('visualize densities')
# densities = np.asarray(densities)
# density_colors = plt.get_cmap('plasma')(
#     (densities - densities.min()) / (densities.max() - densities.min()))
# density_colors = density_colors[:, :3]
# density_mesh = o3d.geometry.TriangleMesh()
# density_mesh.vertices = mesh.vertices
# density_mesh.triangles = mesh.triangles
# density_mesh.triangle_normals = mesh.triangle_normals
# density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
# # o3d.visualization.draw_geometries([density_mesh])

# print('remove low density vertices')
# vertices_to_remove = densities < np.quantile(densities, 0.1)
# mesh.remove_vertices_by_mask(vertices_to_remove)
# print(mesh)
# o3d.visualization.draw_geometries([mesh])

#----------------------------------------------Marching cubes-----------------------------------------------

# voxel = generate_voxel(cloud)

# # smooth = mcubes.smooth(voxel)

# vertices, triangles = mcubes.marching_cubes(voxel, 0)

# mcubes.export_mesh(vertices, triangles, "outputs/scene.dae", "MyScene")

#----------------------------------------------Marching cubes skimage---------------------------------------

# voxel = generate_voxel(cloud)

# verts, faces, normals, values = measure.marching_cubes_lewiner(voxel, 0)

# # Display resulting triangular mesh using Matplotlib. This can also be done
# # with mayavi (see skimage.measure.marching_cubes_lewiner docstring).
# fig = plt.figure(figsize=(10, 10))
# ax = fig.add_subplot(111, projection='3d')

# # Fancy indexing: `verts[faces]` to generate a collection of triangles
# mesh = Poly3DCollection(verts[faces])
# mesh.set_edgecolor('k')
# ax.add_collection3d(mesh)

# ax.set_xlabel("x-axis: a = 6 per ellipsoid")
# ax.set_ylabel("y-axis: b = 10")
# ax.set_zlabel("z-axis: c = 16")

# ax.set_xlim(0, 24)  # a = 6 (times two for 2nd ellipsoid)
# ax.set_ylim(0, 20)  # b = 10
# ax.set_zlim(0, 32)  # c = 16

# plt.tight_layout()
# plt.show()

#----------------------------------------------Write mesh to file-------------------------------------------

# o3d.io.write_triangle_mesh("outputs/output.ply", mesh) 

