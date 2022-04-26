#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError #Bro mellom ROS og openCV
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class error_calculator(object):
    def __init__(self):
        self.opticalFlowLeft_sub = rospy.Subscriber("/optic_flow_mean_vx", Float32, self.opticalFlow_callback_L)
        self.opticalFlowRight_sub = rospy.Subscriber("/optic_flow_mean_vx2", Float32, self.opticalFlow_callback_R)
        self.optical_flow_error_pub = rospy.Publisher ("/optical_flow_error", Float32, queue_size=1)
        self.speed_pub = rospy.Publisher ("/cmd_vel", Twist, queue_size=1)
        self.flowLeft = 0
        self.flowRight = 0
        self.flowError = 0
        self.flowErrorList = [0]


        #Leser utklipp fra venstre del av realsense stream
    def opticalFlow_callback_L(self, data):

        self.flowLeft = data.data


        #Leser utklipp fra hoyre del av realsense stream
    def opticalFlow_callback_R(self, data):
        self.flowRight = data.data

        #Beregner error mellom venstre og hoyre flow, og publiserer i error topic
        self.flowError = abs(self.flowLeft) - abs(self.flowRight);

        #Bruker liste for glatt ut error signal
        self.flowErrorList.append(self.flowError)
        if(len(self.flowErrorList) >5):
            del self.flowErrorList[0]



        #Kalkulerer gjenomsnitt av 10 siste maalinger
        flowErrorAvg = sum(self.flowErrorList) / len(self.flowErrorList)
        self.optical_flow_error_pub.publish(flowErrorAvg)



        #Prover aa sende error inn som twist message
        speed_cmd = Twist()
        speed_cmd.linear.x = 0.8
        speed_cmd.angular.z = flowErrorAvg / 20

        self.speed_pub.publish(speed_cmd)



def main():
    rospy.init_node('optical_flow_errorCalculator_node', anonymous=True)
    error_calculator_object = error_calculator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=='__main__':
    main()
