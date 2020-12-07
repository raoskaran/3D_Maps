"""
This code was written and updated by Christo Aluckal (@christoaluckal)
"""
import os
import fnmatch
import csv, sys

path = sys.argv[1]
files = []

for file_name in os.listdir(path):
    if fnmatch.fnmatch(file_name, '*.txt'):
        files.append(file_name)
for i in files:
    with open(path+i, 'r') as infile, open(path+i[:-4]+".xyz",'w+') as outfile:
        temp = infile.read().splitlines()
        for x in range(0,len(temp)):
            outfile.write(temp[x]+"\n")

# for i in files:
#     with open(i, 'r') as infile, open(i[:-4]+".csv",'w+') as outfile:
#         stripped = (line.strip() for line in infile)
#         lines = (line.split(" ") for line in stripped if line)
#         writer = csv.writer(outfile)
#         writer.writerows(lines)