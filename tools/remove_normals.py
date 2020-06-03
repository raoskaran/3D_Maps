import sys

filename = sys.argv[1]

print(filename)

def point_cloud(inp):
    with open("removed_normals.xyz","w") as fo:
        for line in inp:
            x = line.split()
            fo.write(x[0]+" "+x[1]+" "+x[2]+"\n")

with open(filename,"r") as fo:
    pc = point_cloud(fo)