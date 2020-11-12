#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray,Float32MultiArray

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped

global pubstrimg,pubarrimg,pubarrimg_sd
#pubstrimg = rospy.Publisher('strimg', String, queue_size=10)
pubarrimg = rospy.Publisher('arrimg', Int32MultiArray, queue_size=2)
pubarrimg_sd = rospy.Publisher('arrimg_sd', Int32MultiArray, queue_size=2)
pubYoloPredictions = rospy.Publisher('yolo_predictions_image', Image, queue_size=2)
pub_locOnion_WrldFrame = rospy.Publisher('locOnion_WrldFrame', Float32MultiArray, queue_size=2)

cvb = CvBridge()
arr_img = []

# cv_image=None
 
def __numpy_to_string(A):
    return ','.join(str(x) for x in A)

def __string_to_numpy(S):
    return np.array([int(x) for x in S.split(',')])


def grabrgb(msg):
    try:
        cv_image = cvb.imgmsg_to_cv2(msg, "rgb8")
        print("listener node received hd image from camera topic")
    	arr_img = np.array(cv_image)
    	print(arr_img.shape)
    	flat_arr = np.ravel(arr_img)
    	msg = Int32MultiArray(data=flat_arr)
    	global pubarrimg
    	pubarrimg.publish(msg)
    except CvBridgeError as e:
        print(e)
    return

def grabflatarrimg_rgb(msg):

    flat_arr = np.array(msg.data)
    rospy.loginfo("listener received flat_array of shape %s .",flat_arr.shape)
    # kinectv2
    rgb_int_array = np.reshape(flat_arr,(1080, 1920, 3)).astype(np.uint8)
    # rospy.loginfo("I received arrimg of shape %s . starting a call for prediction.",rgb_mem.shape)
    image_message = cvb.cv2_to_imgmsg(rgb_int_array, encoding="rgb8")
    global pubYoloPredictions
    pubYoloPredictions.publish(image_message)
    return

def grabrgb_sd(msg):
    try:
        cv_image = cvb.imgmsg_to_cv2(msg, "rgb8")
        print("listener node received sd image from camera topic")
        arr_img = np.array(cv_image)
        print(arr_img.shape)
        flat_arr = np.ravel(arr_img)
        msg = Int32MultiArray(data=flat_arr)
        global pubarrimg_sd
        pubarrimg_sd.publish(msg)
    except CvBridgeError as e:
        print(e)
    return

# # parse the cloud into an array
# cloud_arr = np.frombuffer(pcl.data, dtype_list)
# print("cloud_arr ",cloud_arr.shape)
# print("shape after numpify ",numpify(pcl.data).shape)


# counter = 0
# for p in pc2.read_points(pcl, field_names = ("x", "y", "z"), skip_nans=True):
#     counter += 1
#     # print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])    
# print("size from read_points ",counter)
# exit(0)
####### shape of array from ros_numpy.point_cloud2 doesn't match height*width of image
# from ros_numpy import point_cloud2
# pCloudArr = point_cloud2.pointcloud2_to_xyz_array(pcl, remove_nans=True)
# print("pCloudArr ",pCloudArr.shape)
# x,y,z = pCloudArr[point_index,0],pCloudArr[point_index,1],pCloudArr[point_index,2]
##################
# writing data to analyze
# f = open("/home/psuresh/Downloads/pointcloud.txt",'w')
# f.write("")
# f.close()
# f = open("/home/psuresh/Downloads/pointcloud.txt",'a')
# rowcounter = 0
# pcounter = 0
# for point in dataConverted[0:4000*8]:
#     f.write(str(point)+",")
#     pcounter = (pcounter+1) % 8
#     rowcounter = (rowcounter+1) % (8*(pcl.row_step/pcl.point_step))
#     if pcounter == 0:
#         f.write("\npoint\n")

#     if rowcounter == 0:
#         f.write("\n\nrowend\n\n")

# f.close()

# print("shape after fromstring ",dataConverted.shape)    
# print("max value ",np.nanmax(dataConverted))

import sensor_msgs.point_cloud2 as pc2
# from ros_numpy import *
yoloLocations_L2R_KinectFrame = []
import time
import math

#   fit a sphere to X,Y, and Z data points
#   returns the radius and center points of
#   the best fit sphere
# https://jekel.me/2015/Least-Squares-Sphere-Fit/
def sphereFit(spX,spY,spZ):
    rad = 0
    #   Assemble the A matrix
    spX = np.array(spX)
    spY = np.array(spY)
    spZ = np.array(spZ)
    A = np.zeros((len(spX),4))
    A[:,0] = spX*2
    A[:,1] = spY*2
    A[:,2] = spZ*2
    A[:,3] = 1

    #   Assemble the f matrix
    f = np.zeros((len(spX),1))
    f[:,0] = (spX*spX) + (spY*spY) + (spZ*spZ)
    C, residules, rank, singval = np.linalg.lstsq(A,f)

    #   solve for the radius
    t = (C[0]*C[0])+(C[1]*C[1])+(C[2]*C[2])+C[3]
    rad = math.sqrt(t)

    return rad, C[0], C[1], C[2]

import struct
def cbk_retrieveCloudPointsfor4corners(pcl):
    
    # print("recevied cloud length of a row (pcl.row_step, pcl.point_step), height, width, pcl offsets,is_bigendian, is_dense ",\
    #     ((pcl.row_step, pcl.point_step),pcl.height,pcl.width,(pcl.fields[0].offset,\
    #         pcl.fields[1].offset,pcl.fields[2].offset),pcl.is_bigendian, pcl.is_dense))

    global yoloLocations_L2R_KinectFrame

    # # We can derive center of onion by provinding 3 lists of x,y,z values
    average_wdw_sz = 6
    list_x,list_y,list_z = [],[],[]
    cx2d = int(yoloLocations_L2R_KinectFrame[0]) 
    cy2d = int(yoloLocations_L2R_KinectFrame[1]) 
    tlx = int(yoloLocations_L2R_KinectFrame[2]) 
    tly = int(yoloLocations_L2R_KinectFrame[3]) 
    brx = int(yoloLocations_L2R_KinectFrame[4]) 
    bry = int(yoloLocations_L2R_KinectFrame[5]) 

    print("pixel for center of bounding box ",(cx2d,cy2d))
    print("pixels for top-left bottom-right of bounding box ",(tlx,tly,brx,bry))
    for u in range(cx2d-average_wdw_sz/2,cx2d+average_wdw_sz/2+1):
        for v in range(cy2d-average_wdw_sz/2,cy2d+average_wdw_sz/2+1):
            # print("u,v ",u,v)
            # point_idx_x = point_index + 0 #pcl.fields[0].offset
            # point_idx_y = point_index + 1 #pcl.fields[1].offset
            # point_idx_z = point_index + 2 #pcl.fields[2].offset
            # x,y,z = dataConverted[point_idx_x],dataConverted[point_idx_y],dataConverted[point_idx_z]

            point_index = u  + v * (pcl.row_step/pcl.point_step)
            str_point = pcl.data[point_index*pcl.point_step:(point_index+1)*pcl.point_step]
            (x,y,z) = struct.unpack('fff'+'x'*20, str_point)
            print("pixel:",(u,v)," has cloud xyz values ",(x,y,z))
            if not math.isnan(x) and not math.isnan(y) and not math.isnan(z): # and x!= 0.0 and y!=0.0 and z !=0.0:
                list_x.append(x)
                list_y.append(y)
                list_z.append(z)

    if len(list_x) == 0:
        print("all cloud points are Nans on onion surface")
        return 

    array_avgxyz = [np.mean(list_x),np.mean(list_y),np.mean(list_z)]
    print("array_avgxyz ",array_avgxyz)
    vec_2_center = array_avgxyz / np.linalg.norm(array_avgxyz)
    radius = 0.04 # estimate
    appended_xyz = array_avgxyz+radius*vec_2_center
    print("appended_xyz ",appended_xyz)

    camangle_wrt_conveyorplane = 45
    trans_Wrld2Cam = [[1, 0, 0, 0.705],
    [0,-np.cos(np.deg2rad(camangle_wrt_conveyorplane)),np.cos(np.deg2rad(camangle_wrt_conveyorplane)),-0.845],
    [0,-np.sin(np.deg2rad(camangle_wrt_conveyorplane)),-np.sin(np.deg2rad(camangle_wrt_conveyorplane)),0.686],
    [0,0,0,1]]
    xyz_cam = [appended_xyz[0],appended_xyz[1],appended_xyz[2],1]
    [x_w,y_w,z_w] = np.matmul(trans_Wrld2Cam,xyz_cam)[:-1]
    print("transformed coordinates ",[x_w,y_w,z_w])

    pub_locOnion_WrldFrame.publish(Float32MultiArray(data=[x_w,y_w,z_w]))
    return


    rad,cx,cy,cz = sphereFit(list_x,list_z,list_z)
    print("sphere fitting: ",rad,cx,cy,cz)
    if rad > 0:
        exit(0)
        return
    
    # window size of 4 lead to huge variation
    average_wdw_sz = 2

    # for i in range(len(yoloLocations_L2R_KinectFrame)/2):
    #     pointFound = False
    #     # first coordinate in bounding box is along width 1920 of image (> height of image)
    #     # and row of cloud data is same as width
    #     cx = int(yoloLocations_L2R_KinectFrame[2*i]) # along width 
    #     cy = int(yoloLocations_L2R_KinectFrame[2*i+1]) # along height             
    #     point_index = cx  + cy * (pcl.row_step/pcl.point_step)
    #     point_index = u  + v * (pcl.row_step/pcl.point_step)
    #     str_point = pcl.data[point_index*pcl.point_step:(point_index+1)*pcl.point_step]
    #     (x,y,z) = struct.unpack('fff'+'x'*20, str_point)
    #     print("pixel:",(u,v)," has cloud xyz values ",(x,y,z))
        
    #     if not math.isnan(x) and not math.isnan(y) and not math.isnan(z): 
    #         print("at center ",dataConverted[point_idx_x:point_idx_z+1])
    #         pointFound = True 

    #     if not pointFound:
    #         # average by only varying u
    #         sum_x,sum_y,sum_z = 0.0,0.0,0.0
    #         counter = 0.0
    #         v = cy 
    #         for u in range(cx-average_wdw_sz/2,cx+average_wdw_sz/2+1):
    #             point_index = u*8  + v * (pcl.row_step/pcl.point_step)
    #             point_idx_x = point_index + 0 #pcl.fields[0].offset
    #             point_idx_y = point_index + 1 #pcl.fields[1].offset
    #             point_idx_z = point_index + 2 #pcl.fields[2].offset
    #             x,y,z = dataConverted[point_idx_x],dataConverted[point_idx_y],dataConverted[point_idx_z]
    #             print("pixel:",(u,v)," has cloud xyz values ",dataConverted[point_idx_x:point_idx_z+1])
    #             if not math.isnan(x) and not math.isnan(y) and not math.isnan(z): # and x!= 0.0 and y!=0.0 and z !=0.0:
    #                 sum_x,sum_y,sum_z = sum_x+x,sum_y+y,sum_z+z
    #                 counter += 1
    #                 # print((sum_x,sum_y,sum_z),counter)

    #         if counter > 0.0:
    #             x,y,z = sum_x/counter,sum_y/counter,sum_z/counter
    #             print("u - averaged cloud point for onion ",x,y,z)
    #             pointFound = True 

    #         if not pointFound: 
    #             # average all non-NAN 3D coordinates in a small bounding box around centroid
    #             # (considering full box is bad because it will include points on conveyor)
    #             sum_x,sum_y,sum_z = 0.0,0.0,0.0
    #             counter = 0.0
    #             for u in range(cx-average_wdw_sz/2,cx+average_wdw_sz/2+1):
    #                 for v in range(cy-average_wdw_sz/2,cy+average_wdw_sz/2+1):
    #                     # print("u,v ",u,v)
    #                     point_index = u*8  + v * (pcl.row_step/pcl.point_step)
    #                     point_idx_x = point_index + 0 #pcl.fields[0].offset
    #                     point_idx_y = point_index + 1 #pcl.fields[1].offset
    #                     point_idx_z = point_index + 2 #pcl.fields[2].offset
    #                     x,y,z = dataConverted[point_idx_x],dataConverted[point_idx_y],dataConverted[point_idx_z]
    #                     print(dataConverted[point_idx_x:point_idx_z+1])
    #                     if not math.isnan(x) and not math.isnan(y) and not math.isnan(z): # and x!= 0.0 and y!=0.0 and z !=0.0:
    #                         sum_x,sum_y,sum_z = sum_x+x,sum_y+y,sum_z+z
    #                         counter += 1
    #                         # print((sum_x,sum_y,sum_z),counter)

    #             if counter > 0.0:
    #                 x,y,z = sum_x/counter,sum_y/counter,sum_z/counter
    #                 print("u,v averaged cloud point for onion ",x,y,z)
    #                 break

    #     if pointFound:
    #         global trans_QR2Cam,trans_Wrld2QR
    #         [x_qr,y_qr,z_qr] = np.matmul(trans_QR2Cam,np.array([x,y,z,1]))[:-1]
    #         [x_w,y_w,z_w] = np.matmul(trans_Wrld2QR,np.array([x_qr,y_qr,z_qr,1]))[:-1]
    #         print("onion position w.r.t world ",[x_w,y_w,z_w])

        # exit(0)

    return

from std_msgs.msg import Float32MultiArray
def cbk_readLocSequence(msg):
  global yoloLocations_L2R_KinectFrame
  yoloLocations_L2R_KinectFrame = msg.data
  print("received yoloLocations_L2R_KinectFrame ",yoloLocations_L2R_KinectFrame)
  return

from tf.transformations import quaternion_matrix
def cbk_createtrans_QR2Cam(pose_msg):
    global trans_QR2Cam
    p = pose_msg.pose.position
    q = pose_msg.pose.orientation
    trans_Cam2QR = quaternion_matrix([q.w, q.x, q.y, q.z])
    trans_Cam2QR[:,3] = np.array([p.x,p.y,p.z,1])
    # print("trans_Cam2QR  ", trans_Cam2QR)
    trans_QR2Cam = np.linalg.inv(trans_Cam2QR)
    # print("trans_QR2Cam  ", trans_QR2Cam)
    global trans_QR2Cam,trans_Wrld2QR
    [x_qr,y_qr,z_qr] = np.matmul(trans_QR2Cam,np.array([0.01,0.01,0.01,1]))[:-1]
    [x_w,y_w,z_w] = np.matmul(trans_Wrld2QR,np.array([x_qr,y_qr,z_qr,1]))[:-1]
    # print("cam position w.r.t world ",[x_w,y_w,z_w])

    return


# QR is at 0.7393,-0.5141,0.1353 
# bases of frame of QR is: (0,0,-1),(0,1,0),(-1,0,0) 
trans_Wrld2QR = np.array([[0, 0, 1, 0.7393],
                        [0, 1, 0, -0.5141],
                        [-1, 0, 0, 0.1353],
                        [0, 0, 0, 1]])
trans_QR2Cam = None


def listener():

    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber("/kinect2/hd/image_color", Image, grabrgb)
    # rospy.Subscriber("/kinect2/sd/image_color_rect", Image, grabrgb_sd)
    # sawyer head camera with intera.sh
    #rospy.Subscriber("/io/internal_camera/head_camera/image_raw", Image, grabrgb)
    # spin() simply keeps python from exiting until this node is stopped

    rospy.Subscriber('location_ordered_centroids', Float32MultiArray, cbk_readLocSequence)
    rospy.Subscriber("/kinect2/hd/points", PointCloud2, cbk_retrieveCloudPointsfor4corners)
    rospy.Subscriber("/visp_auto_tracker/object_position", PoseStamped, cbk_createtrans_QR2Cam)
    rospy.Subscriber("yolo_predictions_flattened", Float32MultiArray, grabflatarrimg_rgb)
    rospy.spin()

if __name__ == '__main__':

    listener()

