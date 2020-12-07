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

time.sleep(2)

# Initialise all the global variable here

H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
pipe.start(cfg)




xt, yt,zt = [],[],[]


vfov = 58
hfov = 86

global gangle

gangle = 0# <---- First Initialization needed for getpoints()
flag = 1

counter = 0

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
    global flag
    # print("GAngle:",gangle,"Yaw:",yaw)
    if flag is 1:
        flag = 0
        return(yaw,0,640)
    else:
        if gangle > yaw: # CW rotation means yaw increases and in CCW it decreases so the cases are to test whether to add to the left or to the right
            # 0.134 = 86/640 ie hfov/resolution_width, essentially  degree_angle/pixel
            endp = int(abs(abs(gangle)-abs(yaw))//0.134)  #<--- Meaning pixels:0->x
            startp = 0
            if endp==startp:  # <--- If endp becomes 0 then it will not plot anything so cases are added
                startp = 0
                endp = 640
        else:
            startp = 640-int(abs(abs(gangle)-abs(yaw))//0.134) #<--- Meaning pixels: x->640
            endp = 640
            if startp==endp: # <--- Equality case
                startp = 0
                endp = 640

        print(startp,endp,gangle,yaw)
        gangle = yaw # <--- set current angle to gangle for the next iteration

        return (gangle,startp,endp)

def threshold(curr):
    prev = prevPose[-1]
    x1,y1,z1,r1,p1,ya1 = prev
    x2,y2,z2,r2,p2,ya2 = curr
    if abs(x2-x1) > 0.035 or abs(y2-y1) > 0.035 or abs(ya2-ya1) > 5:
        return True
    else:
        print("SKIPPED")
        return False



#Return the Translation Matrix
def get_trans_mat(dx,dy,dz):
    trans = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[dx,dy,dz,1]]
    return trans

def get_rot_mat(theta):
    theta = math.radians(theta)
    rot_mat = [[math.cos(theta),0,-1*math.sin(theta),0],[0,1,0,0],[math.sin(theta),0,math.cos(theta),0],[0,0,0,1]]
    return rot_mat

def homo_matrix(dx,dy,dz,theta):
    theta = math.radians(theta)
    homo_mat = [[math.cos(theta),math.sin(theta),0,0],[-1*math.sin(theta),math.cos(theta),0,0],[0,0,1,0],[dx,dy,dz,1]]
    return homo_mat


def trans_coords(total,trans_mat):
    global fcoords
    global fdepth
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

# def correct_depth(depth_frame,x_pixel):
#     li_1 = []
#     li_2 = []
#     if x_pixel < 20 or x_pixel > 619:
#         return depth_frame.get_distance(x_pixel,240)
#     else:
#         # print("Returned with change for ",x_pixel)
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
    print("Entered Mapget")
    global gangle
    global counter
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


    #Temporary Lists
    coords = [] #Holds the Cartesian X-Coords
    depth = []  #Holds the Cartesian Y-Coords
    tot = [] #Holds the [X,Y] values 
    check = []
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
        
        angle = -yaw #<-- Intel takes CW as +ve and CCW as -ve
        print("Base Angle is: ",angle)
        gangle,startp,endp = getpoints(gangle,yaw)
        for y in range(0,480,20):
            phi1 = (vfov/2)*((240-y)/240)
            phi = math.radians(phi1)
            for x in range(startp,endp,4):
            #get_distance() returns the meter values, but it is not visible in the scatterplot so i multiply it by 100 XD
                depth_val = depth_frame.get_distance(x,y)
                if depth_val == 0 or depth_val > 5:
                    pass
                else:
                    theta_d = (hfov/2)*(320-x)/320
                    
                    #Convert angles to radians
                    theta_d = math.radians(theta_d)
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

                    tot.append([x_dash,y_dash,z_dash,1])

        tx,ty,tz = xx,-1*zz,0
        # transmat = get_trans_mat(tx,ty,tz)

        rot_y = angle
        # rotmat = get_rot_mat(rot_y)

        homo_mat = homo_matrix(tx,ty,tz,rot_y)

        x_val,y_val,z_val = trans_coords(tot,homo_mat)

        for i in range(0,len(x_val)):
            fcoords.append((x_val[i],y_val[i],z_val[i]))
            with open('discrete map/mimo_output_{}.txt'.format(counter), 'a') as the_file:
                the_file.write(str(x_val[i])+" "+str(y_val[i])+" "+str(z_val[i])+"\n")
                the_file.close()

        return fcoords


    finally:
        # Stop streaming
        pipeline.stop()

def mega2():   # <--- Actual iterative updation
    global xt,yt,zt
    global counter
    tempx,tempy,tempz = [],[],[]
    fig = plt.figure()

    ax = fig.add_subplot(111, projection='3d')
    try:
        while True:
            open('discrete map/mimo_output_{}.txt'.format(counter),'w').close()

            with open('discrete map/mimo_output_{}.txt'.format(counter), 'a') as the_file:
                the_file.write("x"+" "+"y"+" "+"z"+" "+"\n")
                the_file.close()

            p = poseGet()
            if True:
                x,y,z,roll,pitch,yaw = p
                # print("Current Yaw:",yaw)
                m = mapGet(p)
                for m in m:
                    xmap,ymap,zmap = m
                    xt.append(xmap)
                    yt.append(ymap)
                    zt.append(zmap)

            counter = counter + 1

                    # else:
                #     pass
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