#!/usr/bin/env python3

import rospy
from sa_net_robo.srv import sanet_srv
from sensor_msgs.msg import Image
import numpy as np
from numpy import save as savenpy
from PIL import Image as im22
from frontend import getStateAction
from cv_bridge import CvBridge, CvBridgeError
import os
import cv2
import time

# os.system("pwd")
import sys
sys.path.append('src/sa_net_robo/')

cvb = CvBridge()

mem_action = 0
mem_x = None
mem_y = None
mem_theta = None
rgb_mem = None
depth_mem = None
same_flag = 0

pred1 = np.asarray([0,0,0,0])
pred2 = np.asarray([0,0,0,0])

gas = getStateAction(rcnch='yolo')


def imnormalize(xmax, image):
    """
    Normalize a list of sample image data in the range of 0 to 1
    : image:image data.
    : return: Numpy array of normalize data
    """
    xmin = 0
    a = 0
    b = 255

    return ((np.array(image, dtype=np.float32) - xmin) * (b - a)) / (xmax - xmin)


def grabrgb(msg):
    global video
    global rgb_mem
    global same_flag
    try:
        cv_image = cvb.imgmsg_to_cv2(msg, "rgb8")

    except CvBridgeError as e:
        print(e)

    image_normal = np.array(cv_image)
    if np.array_equal(rgb_mem, image_normal):
        same_flag = 1
        return
    else:
        rgb_mem = np.copy(image_normal)
        same_flag = 0


def grabdepth(msg):
    global video
    global depth_mem
    global same_flag
    try:
        cv_image = cvb.imgmsg_to_cv2(msg, msg.encoding)
    except CvBridgeError as e:
        print(e)

    image_normal = np.array(imnormalize(np.max(cv_image), cv_image), dtype=np.uint8)
    numpy_image = np.array(cv_image, dtype=np.uint16)

    if (depth_mem == numpy_image).all():
        same_flag = 1
        return
    else:
        depth_mem = np.copy(numpy_image)

def is_number(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def getsa(req):
    global same_flag
    global pred1
    global pred2

    if same_flag == 0:
        import traceback
        try:
            new_im = np.copy(rgb_mem)
            new_im_d = np.copy(depth_mem)

            prediction, lables = gas.getsa_multi(new_im, new_im_d)

            # print('image type::: ',type(new_im))
            
            
            print('pred1 :::: ',pred1)

            if (str(type(prediction[0][4])) != str(type(0))):
                # print('111111111111111111111111111111111')
                p1 = prediction[0][4]
                
                print(str(type(p1)))
                if (type(p1) == type(new_im)):
                    print(p1.shape)
                    print ('p1::: ',p1[0],p1[1],p1[2],p1[3])
                    cv2.rectangle(new_im,(p1[0],p1[1]),(p1[2],p1[3]),(0,255,0),2)
                    if abs(p1[0]-pred1[0])<2 and abs(p1[1]-pred1[1])<2 and abs(p1[2]-pred1[2])<2 and abs(p1[3]-pred1[3])<2:
                        prediction[0][0] = 3

                print('p1::: ',p1)
            if (str(type(prediction[1][4])) != str(type(0))):
                # print('222222222222222222222222222222222')
                p2 = prediction[1][4]
                print(str(type(p2)))

                if (type(p2) == type(new_im)):
                    print(p2.shape)
                    print ('p2::: ',p2[0],p2[1],p2[2],p2[3])
                    cv2.rectangle(new_im,(p2[0],p2[1]),(p2[2],p2[3]),(255,0,0),2)
                    if abs(p2[0]-pred2[0])<2 and abs(p2[1]-pred2[1])<2 and abs(p2[2]-pred2[2])<2 and abs(p2[3]-pred2[3])<2:
                        prediction[1][0] = 3
                print('p2::: ',p2)

            # print('new_im:::: ',type(new_im))
            # # Save the labeled image
            # im = im22.fromarray(new_im)

            
            # ts = time.time() 
            # f_name = './saved_images/' + str(ts) + '.jpg'
            # np_name = './saved_images/' + str(ts) + '.npy'
            # d_name = './saved_images/' + str(ts) + '_d.npy'
            # im.save(f_name)
            # savenpy(np_name, new_im)
            # savenpy(d_name, new_im_d)


            if p1 == -1:
                pred1 = np.asarray([0,0,0,0])
            else:
                pred1 = np.asarray([p1[0],p1[1],p1[2],p1[3]])
            if p2 == -1:
                pred2 = np.asarray([0,0,0,0])
            else:
                pred2 = np.asarray([p2[0],p2[1],p2[2],p2[3]])

            print ('prediction: ',prediction)
            if ( is_number(prediction[0][0])== True and is_number(prediction[1][0])== True):
                return str(prediction[0][0]), str(prediction[0][1]),str(prediction[0][2]),str(prediction[0][3]),str(prediction[1][0]), str(prediction[1][1]),str(prediction[1][2]),str(prediction[1][3])
            elif is_number(prediction[0][0])== True :
                return str(prediction[0][0]), str(prediction[0][1]),str(prediction[0][2]),str(prediction[0][3]), "None", "None", "None", "None"
            elif is_number(prediction[1][0])== True :
                return 'None', "None", "None", "None", str(prediction[1][0]), str(prediction[1][1]), str(prediction[1][2]),str(prediction[1][3]) 
            else:
                return "None","None","None","None","None","None","None","None"
        except:
            traceback.print_exc()


    else:
        # The old values will be used
        pass

'''

when IRL code calls "/get_state_action" service 
it needs to wait until it receives a response
 (state,action) from sa-net service

IRL-node calling some sa-net service
that call starts a callback (getsa) 
that callback calls some function in sa-net code
that will take kinect image as input and return state, action as output

If you start with giving kinect image to yolo instead of this function,
how will rest of code (including irl) align with your code? 



'''

if __name__ == '__main__':
    rospy.init_node("sanet_server")
    # for kinect v2
    rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, grabdepth)
    # for kinect v1
    #rospy.Subscriber("/camera/depth_registered/image_raw", Image, grabdepth)
    # for kinect v2
    rospy.Subscriber("/kinect2/hd/image_color", Image, grabrgb)
    # for kinect v1
    #rospy.Subscriber("/camera/rgb/image_raw", Image, grabrgb)
    rospy.loginfo("sa-net started")
    service = rospy.Service("/get_state_action", sanet_srv, getsa)
    rospy.spin()
