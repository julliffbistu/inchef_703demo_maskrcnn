#!/usr/bin/env python
# coding: utf-8
import rospy
import roslib
from std_msgs.msg import String, Float32MultiArray,Int8
from sensor_msgs import msg
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import math
import sys
import time
ROOT_DIR = os.path.abspath("./")
sys.path.append(ROOT_DIR)
import calibration_class
###calibration class putin 
depth_global=np.zeros([540,960,1],dtype=np.uint16)
railparams=np.zeros(3,dtype=float)
position=int(0)
beef_center_pixelx=300

######publish msgs

resultCenterPose_pan = np.zeros(6,dtype=float)
resultGraspPose_pan_left = np.zeros(6,dtype=float)
resultGraspPose_pan_right = np.zeros(6,dtype=float)

resultCenterPose_beef = np.zeros(6,dtype=float)
resultGraspPose_beef_left = np.zeros(6,dtype=float)
resultGraspPose_beef_right = np.zeros(6,dtype=float)

resultPressPose_beef_one = np.zeros(6,dtype=float)
resultPressPose_beef_two = np.zeros(6,dtype=float)
resultPressPose_beef_three = np.zeros(6,dtype=float)
resultPressPose_beef_four = np.zeros(6,dtype=float)


def callback_depth_img(image):
    bridge=CvBridge()
    cv_image_depth = bridge.imgmsg_to_cv2(image,"passthrough")
    global depth_global
    depth_global = cv_image_depth.copy()
    #print(depth_global.shape)



def callback_position(data):
    global railparams
    global position
    print(":::::::::::::::::::",data)
    railparams=data.data
    print(":::::::::::::::::::",railparams)
    if (abs(railparams[1]-0.8)<0.03):
        position=int(1)
    else:
        position=int(0)



#####callback funcs of beef
def callback_centerpose_beef(data):
    #print("centerpose_beef------u-v:",data.data)
    global beef_center_pixelx
    global position
    u=data.data[0]
    v=data.data[1]
    if u>300:
        position=int(1)
    else:
        position=int(0)

        


    calibra_instance=calibration_class.point_transformation()
    calibra_instance.load_position(position)
    print("}}}}}}}}}}}}}}position:",position)
    resultCenterPose_beef[0],resultCenterPose_beef[1],resultCenterPose_beef[2]=calibra_instance.Pix2baselink(depth_global,u,v)
   # print("..resultCenterPose_beef...../n",resultCenterPose_beef)
    tempResultCenterPose_beef = Float32MultiArray(data = resultCenterPose_beef)
    center_pose_beef_pub = rospy.Publisher("/calbration/centerpose_beef",Float32MultiArray,queue_size=1)
    center_pose_beef_pub.publish(tempResultCenterPose_beef)


def callback_grasp_pose_beef_left(data):
   # print("grasp_pose_beef_left------u-v:",data.data)
    u=data.data[0]
    v=data.data[1]
    calibra_instance=calibration_class.point_transformation()
    calibra_instance.load_position(position)
  #  print("position:",position)
    resultGraspPose_beef_left[0],resultGraspPose_beef_left[1],resultGraspPose_beef_left[2]=calibra_instance.Pix2baselink(depth_global,u,v)
   # print("..resultGraspPose_beef_left...../n",resultGraspPose_beef_left)
    tempresultGraspPose_beef_left = Float32MultiArray(data = resultGraspPose_beef_left)
    grasp_pose_beef_left_pub = rospy.Publisher("/calbration/grasp_pose_beef_left",Float32MultiArray,queue_size=1)
    grasp_pose_beef_left_pub.publish(tempresultGraspPose_beef_left)



def callback_grasp_pose_beef_right(data):
   # print("grasp_pose_beef_right------u-v:",data.data)
    u=data.data[0]
    v=data.data[1]
    calibra_instance=calibration_class.point_transformation()
    calibra_instance.load_position(position)
    #print("position:",position)
    resultGraspPose_beef_right[0],resultGraspPose_beef_right[1],resultGraspPose_beef_right[2]=calibra_instance.Pix2baselink(depth_global,u,v)
  #  print("..resultGraspPose_beef_right...../n",resultGraspPose_beef_right)
    tempresultGraspPose_beef_right = Float32MultiArray(data = resultGraspPose_beef_right)
    grasp_pose_beef_right_pub = rospy.Publisher("/calbration/grasp_pose_beef_right",Float32MultiArray,queue_size=1)
    grasp_pose_beef_right_pub.publish(tempresultGraspPose_beef_right)


def callback_press_pose_beef_one(data):
    #print("press_pose_beef_one------u-v:",data.data)
    u=data.data[0]
    v=data.data[1]
    calibra_instance=calibration_class.point_transformation()
    calibra_instance.load_position(position)
    #print("position:",position)
    resultPressPose_beef_one[0],resultPressPose_beef_one[1],resultPressPose_beef_one[2]=calibra_instance.Pix2baselink(depth_global,u,v)
   # print("..resultPressPose_beef_one.....",resultPressPose_beef_one)

    tempresultPressPose_beef_one = Float32MultiArray(data = resultPressPose_beef_one)
    press_pose_beef_one_pub = rospy.Publisher("/calbration/press_pose_beef_one",Float32MultiArray,queue_size=1)
    press_pose_beef_one_pub.publish(tempresultPressPose_beef_one)

def callback_press_pose_beef_two(data):
   # print("press_pose_beef_two------u-v:",data.data)
    u=data.data[0]
    v=data.data[1]
    calibra_instance=calibration_class.point_transformation()
    calibra_instance.load_position(position)
    #print("position:",position)
    resultPressPose_beef_two[0],resultPressPose_beef_two[1],resultPressPose_beef_two[2]=calibra_instance.Pix2baselink(depth_global,u,v)
   # print("..resultPressPose_beef_two.....",resultPressPose_beef_two)

    tempresultPressPose_beef_two = Float32MultiArray(data = resultPressPose_beef_two)
    press_pose_beef_two_pub = rospy.Publisher("/calbration/press_pose_beef_two",Float32MultiArray,queue_size=1)
    press_pose_beef_two_pub.publish(tempresultPressPose_beef_two)

def callback_press_pose_beef_three(data):
   # print("press_pose_beef_three------u-v:",data.data)
    u=data.data[0]
    v=data.data[1]
    calibra_instance=calibration_class.point_transformation()
    calibra_instance.load_position(position)
   # print("position:",position)
    resultPressPose_beef_three[0],resultPressPose_beef_three[1],resultPressPose_beef_three[2]=calibra_instance.Pix2baselink(depth_global,u,v)
    #print("..resultPressPose_beef_three.....",resultPressPose_beef_three)
    tempresultPressPose_beef_three = Float32MultiArray(data = resultPressPose_beef_three)
    press_pose_beef_three_pub = rospy.Publisher("/calbration/press_pose_beef_three",Float32MultiArray,queue_size=1)
    press_pose_beef_three_pub.publish(tempresultPressPose_beef_three)



def callback_press_pose_beef_four(data):
    #print("press_pose_beef_four------u-v:",data.data)
    u=data.data[0]
    v=data.data[1]
    calibra_instance=calibration_class.point_transformation()
    calibra_instance.load_position(position)
   # print("position:",position)
    resultPressPose_beef_four[0],resultPressPose_beef_four[1],resultPressPose_beef_four[2]=calibra_instance.Pix2baselink(depth_global,u,v)
    #print("..resultPressPose_beef_four.....",resultPressPose_beef_four)
    tempresultPressPose_beef_four = Float32MultiArray(data = resultPressPose_beef_four)
    press_pose_beef_four_pub = rospy.Publisher("/calbration/press_pose_beef_four",Float32MultiArray,queue_size=1)
    
    press_pose_beef_four_pub.publish(tempresultPressPose_beef_four)

####callback funcs of pan

def callback_centerpose_pan(data):
    #print("centerpose_pan------u-v:",data.data)
    u=data.data[0]
    v=data.data[1]
    calibra_instance=calibration_class.point_transformation()
    calibra_instance.load_position(position)
    #print("position:",position)
    resultCenterPose_pan[0],resultCenterPose_pan[1],resultCenterPose_pan[2]=calibra_instance.Pix2baselink(depth_global,u,v)
    tempResultCenterPose_pan = Float32MultiArray(data = resultCenterPose_pan)
    #print("..tempResultCenterPose_pan.....",tempResultCenterPose_pan)
    center_pose_pan_pub = rospy.Publisher("/calbration/centerpose_pan",Float32MultiArray,queue_size=1)
    center_pose_pan_pub.publish(tempResultCenterPose_pan)


def callback_grasp_pose_pan_left(data):
    #print("grasp_pose_pan_left------u-v:",data.data)
    u=data.data[0]
    v=data.data[1]
    calibra_instance=calibration_class.point_transformation()
    calibra_instance.load_position(position)
    #print("position:",position)
    resultGraspPose_pan_left[0],resultGraspPose_pan_left[1],resultGraspPose_pan_left[2]=calibra_instance.Pix2baselink(depth_global,u,v)
    #print("..resultGraspPose_pan_left.....",resultGraspPose_pan_left)
    tempresultGraspPose_pan_left = Float32MultiArray(data = resultGraspPose_pan_left)
    grasp_pose_pan_left_pub = rospy.Publisher("/calbration/grasp_pose_pan_left",Float32MultiArray,queue_size=1)
    grasp_pose_pan_left_pub.publish(tempresultGraspPose_pan_left)



def callback_grasp_pose_pan_right(data):
    #print("grasp_pose_pan_right------u-v:",data.data)
    u=data.data[0]
    v=data.data[1]
    calibra_instance=calibration_class.point_transformation()
    calibra_instance.load_position(position)
    print("position:",position)
    resultGraspPose_pan_right[0],resultGraspPose_pan_left[1],resultGraspPose_pan_left[2]=calibra_instance.Pix2baselink(depth_global,u,v)
    #print("..resultGraspPose_pan_right.....",resultGraspPose_pan_right)
    tempresultGraspPose_pan_right = Float32MultiArray(data = resultGraspPose_pan_right)
    grasp_pose_pan_right_pub = rospy.Publisher("/calbration/grasp_pose_pan_right",Float32MultiArray,queue_size=1)
    grasp_pose_pan_right_pub.publish(tempresultGraspPose_pan_right)







rate=20

def spinOnce():
    print("::::::::::::rate")
    rate=10
    r = rospy.Rate(rate)
    r.sleep()


def listener():
    rospy.init_node("calibration_node")
    rospy.Subscriber("/maskrcnn/depth_img",msg.Image,callback_depth_img,queue_size=1)
    ##rail_position
    rospy.Subscriber("/rail/position_temp",Float32MultiArray,callback_position,queue_size=1)
    #beef
    rospy.Subscriber("maskrcnn/centerpose_beef",Float32MultiArray,callback_centerpose_beef,queue_size=1)

    rospy.Subscriber("/maskrcnn/grasp_pose_beef_left",Float32MultiArray,callback_grasp_pose_beef_left,queue_size=1)
    rospy.Subscriber("/maskrcnn/grasp_pose_beef_right",Float32MultiArray,callback_grasp_pose_beef_right,queue_size=1)
    rospy.Subscriber("/maskrcnn/press_pose_beef_one",Float32MultiArray,callback_press_pose_beef_one,queue_size=1)
    rospy.Subscriber("/maskrcnn/press_pose_beef_two",Float32MultiArray,callback_press_pose_beef_two,queue_size=1)
    rospy.Subscriber("/maskrcnn/press_pose_beef_three",Float32MultiArray,callback_press_pose_beef_three,queue_size=1)
    rospy.Subscriber("/maskrcnn/press_pose_beef_four",Float32MultiArray,callback_press_pose_beef_four,queue_size=1)
    
    #pan
    rospy.Subscriber("/maskrcnn/centerpose_pan",Float32MultiArray,callback_centerpose_pan,queue_size=1)
    rospy.Subscriber("/maskrcnn/grasp_pose_pan_left",Float32MultiArray,callback_grasp_pose_pan_left,queue_size=1)
    rospy.Subscriber("/maskrcnn/grasp_pose_pan_right",Float32MultiArray,callback_grasp_pose_pan_right,queue_size=1)
    '''
    rail=np.zeros(3,dtype=float)
    rail_topic = Float32MultiArray(data = rail)

    pub = rospy.Publisher('/topic_name', Float32MultiArray, queue_size=10)
    time.sleep(2)
    pub.publish(rail_topic)
    spinOnce()
    '''
    rospy.spin()
    #

if __name__ == '__main__':  
    listener()

    cv2.destroyAllWindows()



