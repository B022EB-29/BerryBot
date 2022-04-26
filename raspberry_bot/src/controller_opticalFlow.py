#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image #For aa kunne lese data fra realsense kamera
from cv_bridge import CvBridge, CvBridgeError #Bro mellom ROS og openCV
import cv2
import numpy as np
from geometry_msgs.msg import Twist #For aa kunne skrive cmd_vel kommandoer

class LineFollower_edgeDetection(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/realsense/color/image_raw", Image, self.camera_callback) #Subscriber til realsense bilde stream
        self.bridge_object = CvBridge() #Opretter cv bridge objekt
        self.opticalFlow_left_crop_pub = rospy.Publisher("/raspberry_bot/opticalFlow/left_crop", Image, queue_size=1)
        self.opticalFlow_right_crop_pub = rospy.Publisher("/raspberry_bot/opticalFlow/right_crop", Image, queue_size=1)


    def camera_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") #Konverterer bilde til OpenCv format
        except CvBridgeError as e:
            print(e)

        #Klipper ut stream fra hoyre og venstre rad
        #Total height: 480, width: 640
        left_crop = cv_image[250:320,1:140]
        right_crop = cv_image[250:320,500:640]


        #Publiserer bilder slik at egen init_node kan beregne optic flow i bildene
        try:
            opticalFlow_left_crop_msgsFormat = self.bridge_object.cv2_to_imgmsg(left_crop, "bgr8")
            opticalFlow_right_crop_msgsFormat = self.bridge_object.cv2_to_imgmsg(right_crop, "bgr8")
            self.opticalFlow_left_crop_pub.publish(opticalFlow_left_crop_msgsFormat)
            self.opticalFlow_right_crop_pub.publish(opticalFlow_right_crop_msgsFormat)
        except CvBridgeError as e:
            print(e)


def main():
    rospy.init_node('line_follower_opticalFlow_node', anonymous=True)
    line_follower_edgeDetection_objetct = LineFollower_edgeDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=='__main__':
    main()
