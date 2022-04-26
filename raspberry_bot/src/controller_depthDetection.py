#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image #For aa kunne lese data fra realsense kamera
from cv_bridge import CvBridge, CvBridgeError #Bro mellom ROS og openCV
import cv2
import numpy as np
from geometry_msgs.msg import Twist #For aa kunne skrive cmd_vel kommandoer

class LineFollower_edgeDetection(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.camera_callback) #Subscriber til realsense bilde stream
        self.bridge_object = CvBridge() #Opretter cv bridge objekt
        self.speed_pub = rospy.Publisher ("/cmd_vel", Twist, queue_size=1) #cmd_vel kommando Publisher
        self.cannyImg_pub = rospy.Publisher("/raspberry_bot/canny_img", Image, queue_size=1)


    def camera_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="passthrough") #Konverterer bilde til OpenCv format
        except CvBridgeError as e:
            print(e)



        #Skalerer ned bilde for a spare datakraft
        height, width = cv_image.shape
        descentre = 100
        rows_to_watch = 60
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]

        upper_intensity = np.array([5])
        lower_intensity = np.array([0])

        #gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #canny = cv2.Canny(crop_img, 50, 50)
        mask = cv2.inRange(crop_img, lower_intensity, upper_intensity)

        converterd_bitwiseNot = cv2.bitwise_not(mask)


        #Publiserer bilde med kanter
        try:
            canny_sensor_msgsFormat = self.bridge_object.cv2_to_imgmsg(mask, "8UC1")
            self.cannyImg_pub.publish(canny_sensor_msgsFormat)
        except CvBridgeError as e:
            print(e)

        #Kalkulerer midtpunkt paa kjorefelt i masket bilde
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2

        #Enkel P kontroller for kjoring langs midten av kjorefelt
        error_x = cx - width / 2;
        speed_cmd = Twist();
        speed_cmd.linear.x = 2;
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
