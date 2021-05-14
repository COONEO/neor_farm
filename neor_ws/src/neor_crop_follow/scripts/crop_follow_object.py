#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge ,CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class CropFollower(object):
    def __init__(self):
        self.cvBridge = CvBridge()
        self.sub_left_camera = rospy.Subscriber('/L_camera/L_image_raw', Image, self.camera_callback, queue_size = 1)
        #self.sub_right_camera = rospy.Subscriber('/R_camera/image_raw', Image, self.camera_callback, queue_size = 1)
        self.flag = 1
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 10)

    # ROS Image's topic callback function
    def camera_callback(self, image_msg): 
        try:
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
                print(e)
        cv2.imshow("cv_imcode_image",cv_image)

        height, width, channels = cv_image.shape        #Gazebo L_camera default resolution is [640,480]
        #crop_img = cv_image.copy()
        crop_img = cv_image[0 : (height)/2] [0:width]   

        #convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img,cv2.COLOR_BGR2HSV)
        #cv2.imshow("HSV",hsv)

        # green colour in HSV
        lower_color = np.array([56,70,0])
        upper_color = np.array([72,255,255])

        #Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv,lower_color,upper_color)
        #cv2.imshow("MASK",mask)
        
        #Bitwise-and musk and original image
        res = cv2.bitwise_and(crop_img,crop_img,mask = mask)
        cv2.imshow("RES",res)
  
        #Calculate centroid of the blob of binary image using imageMoments
        m = cv2.moments(mask,False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
                cx, cy = height/2, width/2
                self.flag = 0
        '''
        # Draw the centroid in the resultut image
        # start and end point 
        ptStart = ((height*3)/8, 0)
        ptEnd = ((height*3)/8, width)
        point_color = (0, 255, 0) # BGR
        thickness = 1 
        lineType = 4
        cv2.line(crop_img, ptStart, ptEnd, point_color, thickness, lineType)
        cv2.circle(res , (int(cx) , int(cy)) , 20 , (255,255,0) , 2)
        '''
        if cy > (height/2):
            self.flag = 0
        else:
            error_y = height/4 - cy        # the center point pix is height/5  

        if self.flag == 0 :
            twist_object = Twist()
            twist_object.linear.x = 0
            twist_object.angular.z = 0
            rospy.loginfo("match error")
            self.cmd_vel_pub.publish(twist_object)
        else:
            twist_object = Twist()
            twist_object.linear.x = 0.1
            twist_object.angular.z = error_y/70          # if angular.z > 0,and neor will turn left
            rospy.loginfo("ANGULAR VALUE SENT ===>"+str(twist_object.angular.z))
            self.cmd_vel_pub.publish(twist_object)      
        self.flag = 1    
        
        cv2.waitKey(30)

def main():
    rospy.init_node('crop_follow_object',anonymous=True)
    crop_follow_object = CropFollower()
    ctrl_c = False
    rate = rospy.Rate(5)
    while not ctrl_c:
        rate.sleep();

if __name__ == '__main__':
    main()
