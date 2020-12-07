## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################
"""
This code was written and updated by Christo Aluckal (@christoaluckal)
"""
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D 

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


#Temporary Lists
tot = []
#Final Lists
x_coords = []
y_coords = []
z_coords = []

#Horizontal FOV of the D435
hfov = 86
vfov = 58

#Return the Translation Matrix
def get_trans_mat(dx,dy):
    trans = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[dx,dy,0,1]]
    return trans


def trans_coords(total,trans_mat):
    global x_coords
    global z_coords
    global y_coords
    transmat = np.asarray(trans_mat)
    base = np.asarray(total)    #This is the Numpy array of the coordinates before translation
    ans = np.matmul(base,transmat)
    answer = ans.tolist()       #Converting from Numpy array to list of lists

    x0,y0,z0,ones = zip(*answer)   #Extract each element of the lists
    x_coords = x_coords + list(x0)
    y_coords = y_coords + list(y0)
    z_coords = z_coords + list(z0)



# Start streaming
pipeline.start(config)



try:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    # color_frame = frames.get_color_frame()
    # depth_image = np.asanyarray(depth_frame.get_data())
    # color_image = np.asanyarray(color_frame.get_data())

    #Angle input in degrees
    # angle = int(input('Enter angle:'))
    angle = 0
    for y in range(0,480,20):
        phi1 = (vfov/2)*((240-y)/240)
        phi = math.radians(phi1)
        for x in range(0,640,5):
        #get_distance() returns the meter values, but it is not visible in the scatterplot so i multiply it by 100 XD
            depth_val = depth_frame.get_distance(x,y)
            # To ignore 0 depth values
            if depth_val == 0:
                pass

            else:
                # angle: angle rotated by the camera assuming +Y axis as 0 degrees, +90 is to shift the "heading" from +X axis to +Y axis
                # assuming x values range from 0 to 639, theta is the angle made by a pixel location wrt +X axis
                # theta1 = (angle+90+(hfov/2))*((320-x)/320)
                
                theta_d = (hfov/2)*(320-x)/320
                
                #Convert angles to radians
                print(theta_d)
                theta_d = math.radians(theta_d)
                print(theta_d,"\n")

                if theta_d is not 0 and phi is not 0:

                    y_dash = depth_val
                    x_dash = -1*depth_val*math.tan(theta_d)
                    z_dash = depth_val*math.tan(phi)
                    
                else:
                    if theta_d is 0:
                        x_dash = 0
                        z_dash = depth_val*math.tan(phi)
                        y_dash = depth_val

                    elif phi is 0:
                        x_dash = -1*depth_val*math.tan(theta_d)
                        y_dash = depth_val
                        z_dash = 0


                #(X,Y,Z,1) is required for homogenous transformation
                tot.append([x_dash,y_dash,z_dash,1])
            
    tx,ty = 0,0
    transmat = get_trans_mat(tx,ty)

    trans_coords(tot,transmat)
    



    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')


    ax.scatter(x_coords,y_coords,z_coords,s=1)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()

    open('siso_output.txt','w+').close()

    with open('siso_output.txt','w+') as op:
        op.write("x"+" "+"y"+" "+"z"+"\n")
        for iterator in range(0,len(x_coords)):
            op.write(str(x_coords[iterator])+" "+str(y_coords[iterator])+" "+str(z_coords[iterator])+"\n")


finally:
    # Stop streaming
    pipeline.stop()