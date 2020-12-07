"""
This code was written and updated by Christo Aluckal (@christoaluckal)
"""
from matplotlib import pyplot as plt
import numpy as np
import pyrealsense2 as rs
import time
import transformations as tf
import math
from mpl_toolkits.mplot3d import Axes3D

time.sleep(2) #Provide some time to hold the camera. Optional

open('miso_output.txt','w+').close() # Creates new txt file or overwrites and wipes all data in existing txt

with open('miso_output.txt', 'a') as the_file: # Basic appending for the xyz file format
    the_file.write(str(0)+" "+str(0)+" "+str(0)+"\n")

# Initialise all the global variable here

H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
pipe.start(cfg)
blocksize = 2

obs_val = []

xt, yt,zt = [],[],[]

first_x = 0
first_y = 0
last_x = 0
last_y = 0

test_list = []

y_list = []

vfov = 58
hfov = 86

global gangle

gangle = 0 # <---- First Initialization needed for getpoints()
roi_flag = 1 # Global variable that signifies if its the first iteration. Used only for the calculation of the ROI
correction_flag = 0 # Another global variable that signifies if its the first iteration. Used for correction of the pointsets.

def poseGet():
    # Returns a tuple of (x,y,z,roll,pitch.yaw)
    try:
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        # Fetch pose frame
        pose = frames.get_pose_frame()
        if pose:
            # Print some of the pose data to the terminal
            data = pose.get_pose_data()
            #print("Frame #{}".format(pose.frame_number))
            #print("Position: {}".format(data.translation))
            #print("Velocity: {}".format(data.velocity))
            #print("Acceleration: {}\n".format(data.acceleration))        
            
            #(data.translation.x,data.translation.y,data.translation.z)
            
            H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x,data.rotation.y,data.rotation.z]) # in transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!

            # transform to aeronautic coordinates (body AND reference frame!)
            H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody ))

            rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'rxyz') )

            return (data.translation.x,data.translation.y,data.translation.z,(rpy_rad*180/math.pi)[0],(rpy_rad*180/math.pi)[1],(rpy_rad*180/math.pi)[2])
    finally:
        #pipe.stop()
        pass



def getpoints(gangle,yaw):
    global roi_flag
    # print("GAngle:",gangle,"Yaw:",yaw)
    if roi_flag is 1:
        roi_flag = 0
        return(yaw,0,640)
    else:
        if gangle > yaw: # CW rotation means yaw increases and in CCW it decreases so the cases are to test whether to add to the left or to the right
            # 0.134 = 86/640 ie hfov/resolution_width, essentially  degree_angle/pixel
            pixel_end = int(abs(abs(gangle)-abs(yaw))//0.134)  #<--- Meaning pixels:0->x
            pixel_start = 0
            if pixel_end==pixel_start:  # <--- If pixel_end becomes 0 then it will not plot anything so cases are added
                pixel_start = 0
                pixel_end = 640
        else:
            pixel_start = 640-int(abs(abs(gangle)-abs(yaw))//0.134) #<--- Meaning pixels: x->640
            pixel_end = 640
            if pixel_start==pixel_end: # <--- Equality case
                pixel_start = 0
                pixel_end = 640

        # print(pixel_start,pixel_end,gangle,yaw)
        gangle = yaw # <--- set current angle to gangle for the next iteration

        return (gangle,pixel_start,pixel_end)

# def threshold(curr):
#     prev = prevPose[-1]
#     x1,y1,z1,r1,p1,ya1 = prev
#     x2,y2,z2,r2,p2,ya2 = curr
#     if abs(x2-x1) > 0.035 or abs(y2-y1) > 0.035 or abs(ya2-ya1) > 5:
#         return True
#     else:
#         print("SKIPPED")
#         return False



#Return the Translation Matrix
# def get_trans_mat(dx,dy,dz):
#     trans = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[dx,dy,dz,1]]
#     return trans

# def get_rot_mat(theta):
#     theta = math.radians(theta)
#     rot_mat = [[math.cos(theta),0,-1*math.sin(theta),0],[0,1,0,0],[math.sin(theta),0,math.cos(theta),0],[0,0,0,1]]
#     return rot_mat

def homo_matrix(dx,dy,dz,theta):
    theta = math.radians(theta)
    homo_mat = [[math.cos(theta),math.sin(theta),0,0],[-1*math.sin(theta),math.cos(theta),0,0],[0,0,1,0],[dx,dy,dz,1]]
    return homo_mat


def trans_coords(total,trans_mat):
    transmat = np.asarray(trans_mat)
    base = np.asarray(total)    #This is the Numpy array of the coordinates before translation
    ans = np.matmul(base,transmat)
    answer = ans.tolist()       #Converting from Numpy array to list of lists

    x0,y0,z0,ones = zip(*answer)   #Extract each element of the lists
    return (x0,y0,z0)

def rot_coords(total,rot_mat):
    rotmat = np.asarray(rot_mat)
    base = np.asarray(total)    #This is the Numpy array of the coordinates before translation
    ans = np.matmul(base,rot_mat)
    answer = ans.tolist()       #Converting from Numpy array to list of lists

    x0,y0,z0,ones = zip(*answer)   #Extract each element of the lists
    return (x0,y0,z0)

# FUNCTION TO CORRECT DEPTH INCASE IT IS 0 OR GREATER THAN A SPECIFIC NUMBER. OPTIONAL FUNCTION
# def correct_depth(depth_frame,x_pixel):
#     li_1 = []
#     li_2 = []
#     if x_pixel < 20 or x_pixel > 619:
#         return depth_frame.get_distance(x_pixel,240)
#     else:
#         for i in range(1,21):
#             li_1.append(depth_frame.get_distance((x_pixel-i),240))
#             li_2.append(depth_frame.get_distance((x_pixel+i),240))

#         li_1 = np.array(li_1)
#         li_2 = np.array(li_2)

#         # print(li_1,li_1[np.nonzero(li_1)])
#         average_1 = li_1[np.nonzero(li_1)].mean()
#         average_2 = li_2[np.nonzero(li_2)].mean()

#         final = average_1+average_2+depth_frame.get_distance(x_pixel,240)
#         final = final/3
#         if math.isnan(final) is True or final > 6:
#             final = 0
#             # print("Final Depth for ",x_pixel,"is ",final)
#             return 0
#         else:
#             # print("Final Depth for ",x_pixel,"is ",final)
#             return final

def mapGet(postuple):
    pixel_count = 0 # Temporary variable that counts how many pixels are plotted each row.
    pixel_count_list = [] # List that holds individual pixel counts. Default size is 24.
    print("Entered Mapget")
    global gangle
    global correction_flag
    global roi_flag
    global first_x,first_y,last_x,last_y
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


    #Temporary Lists
    tot = [] #Holds the [X,Y] values 
    fcoords = []


    #Horizontal FOV of the D435
    hfov = 86


    # Start streaming
    pipeline.start(config)

    try:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        xx,yy,zz,rol,pit,yaw = postuple
        angle = -yaw #<-- Intel takes CW as +ve and CCW as -ve. This is the angle the camera makes.
        print("Base Angle is: ",angle)
        if roi_flag is 0:
            gangle,pixel_start,pixel_end = getpoints(gangle,yaw)
        else:
            roi_flag = 0
            gangle,pixel_start,pixel_end = (yaw,0,640)

        for y in range(0,480,20):
            temp_phi = (vfov/2)*((240-y)/240)
            phi = math.radians(temp_phi)
            for x in range(pixel_start,pixel_end,5):
                depth_val = depth_frame.get_distance(x,y)
                if depth_val == 0 or depth_val > 5:
                    pass
                else:
                    pixel_count = pixel_count+1
                    theta = (hfov/2)*(320-x)/320
                    if theta is not 0 and phi is not 0:
                        theta = math.radians(theta)
                        y_dash = depth_val
                        x_dash = -1*depth_val*math.tan(theta)
                        z_dash = depth_val*math.tan(phi)
                    else:
                        if theta is 0:
                            x_dash = 0
                            z_dash = depth_val*math.tan(phi)
                            y_dash = depth_val

                        elif phi is 0:
                            x_dash = -1*depth_val*math.tan(theta)
                            y_dash = depth_val
                            z_dash = 0

                    tot.append([x_dash,y_dash,z_dash,1])

            pixel_count_list.append(pixel_count)
            pixel_count = 0

        tx,ty,tz = xx,-1*zz,0

        rot_y = angle

        homo_mat = homo_matrix(tx,ty,tz,rot_y)

        x_val,y_val,z_val = trans_coords(tot,homo_mat)

        pixel_count_top_half_row = np.array(pixel_count_list[0:12])
        pixel_count_sum = np.sum(pixel_count_top_half_row)
        pixel_count_sum_2 = pixel_count_sum+pixel_count_list[12]-1

        if correction_flag is 0:
            last_x,last_y = x_val[pixel_count_sum_2],y_val[pixel_count_sum_2]
            correction_flag = 1
        else:
            first_x,first_y = x_val[pixel_count_sum],y_val[pixel_count_sum]
            offset_x = last_x-first_x
            offset_y = last_y-first_y
            x_val = np.array(x_val)
            x_val = x_val+offset_x
            y_val = np.array(y_val)
            y_val = y_val+offset_y
            x_val = list(x_val)
            y_val = list(y_val)
            last_x,last_y = x_val[pixel_count_sum_2],y_val[pixel_count_sum_2]


        for i in range(0,len(x_val)):
            fcoords.append((x_val[i],y_val[i],z_val[i]))
            with open('miso_output.txt', 'a') as the_file:
                    the_file.write(str(x_val[i])+" "+str(y_val[i])+" "+str(z_val[i])+"\n")

        return fcoords


    finally:
        
        # Stop streaming
        pipeline.stop()

def mega2():   # <--- Actual iterative updation
    global xt,yt,zt
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    try:
        while True:
            p = poseGet()
            if True:
                x,y,z,roll,pitch,yaw = p
                m = mapGet(p)
                for m in m:
                    xmap,ymap,zmap = m
                    xt.append(xmap)
                    yt.append(ymap)
                    zt.append(zmap)

    except KeyboardInterrupt:
        ax.scatter(xt,yt,zt,s=1)
        plt.show()
        plt.close()
        p = poseGet()
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')


def main():
    mega2() # <-- Stupid stuff, calls megaPlot() once then calls mega2()
    pipe.stop()
    plt.waitforbuttonpress()

if __name__ == "__main__":
    gangle = 0
    prevPose = []
    prevPose.append(poseGet())
    main()