#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image #For aa kunne lese data fra realsense kamera
from cv_bridge import CvBridge, CvBridgeError #Bro mellom ROS og openCV
import cv2
import numpy as np
from geometry_msgs.msg import Twist #For aa kunne skrive cmd_vel kommandoer

class LineFollower_edgeDetection(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback) #Subscriber til realsense bilde stream
        self.bridge_object = CvBridge() #Opretter cv bridge objekt
        self.speed_pub = rospy.Publisher ("/cmd_vel", Twist, queue_size=1) #cmd_vel kommando Publisher
        self.cannyImg_pub = rospy.Publisher("/raspberry_bot/canny_img", Image, queue_size=1)


    def camera_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") #Konverterer bilde til OpenCv format
        except CvBridgeError as e:
            print(e)

        #Skalerer ned bilde for a spare datakraft
        height, width, channels = cv_image.shape
        descentre = 100
        rows_to_watch = 60
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]


        #Blurer bildet for aa redusere antall kanter som blir detektert i den neste algoritmen
        blur = cv2.GaussianBlur(crop_img, (5,5), cv2.BORDER_DEFAULT)

        #Detekterer kanter i bilde
        canny = cv2.Canny(blur, 50, 175)

        #Gjor kantene tydligere/tykkere ved hjelp av dilate
        dilated = cv2.dilate(canny, (7,7), iterations=7)

        #Prover aa bruke konturer
        #gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #blur_gray = cv2.GaussianBlur(gray, (5,5), cv2.BORDER_DEFAULT)
        #canny_gray = cv2.Canny(blur_gray, 50, 175)
        #ret, thresh = cv2.threshold(gray, 125, 255, cv2.THRESH_BINARY)

        #Konverterer bildet faar aa kunne bruke samme kalkulasjonsalgoritme som
        #i controller_rgbFilter
        converterd_bitwiseNot = cv2.bitwise_not(dilated)


        #Publiserer bilde med kanter
        try:
            canny_sensor_msgsFormat = self.bridge_object.cv2_to_imgmsg(converterd_bitwiseNot, "8UC1")
            self.cannyImg_pub.publish(canny_sensor_msgsFormat)
        except CvBridgeError as e:
            print(e)

        #Kalkulerer midtpunkt paa kjorefelt i masket bilde
        m = cv2.moments(converterd_bitwiseNot, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2

        #Enkel P kontroller for kjoring langs midten av kjorefelt
        error_x = cx - width / 2;
        speed_cmd = Twist();
        speed_cmd.linear.x =0.5;
        speed_cmd.angular.z = -error_x / 100;

        #self.speed_pub.publish(speed_cmd)






def main():
    rospy.init_node('line_follower_edgeDetection_node', anonymous=True)
    line_follower_edgeDetection_objetct = LineFollower_edgeDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=='__main__':
    main()
