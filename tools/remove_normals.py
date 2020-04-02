def point_cloud(inp):
    with open("eiffel1.xyz","w") as fo:
        for line in inp:
            x = line.split()
            fo.write(x[0]+" "+x[1]+" "+x[2]+"\n")

with open("eiffel.xyz","r") as fo:
    pc = point_cloud(fo)