import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from mpl_toolkits.mplot3d import Axes3D
import triangle as tr

def point_cloud(inp):
    node = []
    for line in inp:
        temp1 = line.strip()
        x = temp1.split()
        node.append(x)
    # print(node)
    node = set(map(tuple,node))
    return node

with open("data/chair.xyz","r") as fo:
    pc = point_cloud(fo)

u = []
v = []
w = []

for l in pc:
    if not l[0].isalpha():
        u.append(float(l[0]))
        v.append(float(l[1]))
        w.append(float(l[2])) 

ua = np.array(u)
va = np.array(v)

#tri = mtri.Triangulation(u, v)
tri = Delaunay(np.array([u,v]).T)

points3d = []
points2d = []
vertex = []

for i in range(ua.shape[0]):
    points3d.append([ua[i],va[i],w[i]])
    points2d.append([ua[i],va[i]])


for vert in tri.simplices:
#for vert in tri.triangles:
    vertex.append(vert)   

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, projection='3d')
ax.plot_trisurf(ua, va, w, triangles=tri.simplices, cmap=plt.cm.Spectral)
#ax.plot_trisurf(ua, va, w, triangles=tri.triangles, cmap=plt.cm.Spectral)
plt.show()