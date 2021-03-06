#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, LaserScan #For aa kunne lese data fra realsense kamera
import numpy as np
from geometry_msgs.msg import Twist #For aa kunne skrive cmd_vel kommandoer


class RowCentering_pointcl2lasersc(object):
    def __init__(self):
        self.speed_pub = rospy.Publisher ("/cmd_vel", Twist, queue_size=1) #cmd_vel kommando Publisher


        self.LaserScan_sub = rospy.Subscriber("/realsense/scan", LaserScan, self.LaserScan_callback)
        self.LaserScan_data = None
        self.LaserScan_data_average = 0
        self.linearPaadrag = 0.0
        self.LaserScan_leftrow = []
        self.LaserScan_front = []
        self.LaserScan_leftrow_avrg = 0
        self.LaserScan_front_avrg = 0

    def LaserScan_callback(self,data):
        #Henter ut tabellen med laser straale og deler den opp i straaler for
        #hoyre og venstre rad. regner deretter ut snittet av tabellene
        self.LaserScan_data = data.ranges
        self.LaserScan_leftrow = self.LaserScan_data[len(self.LaserScan_data)-5:]
        self.LaserScan_front = self.LaserScan_data[40:50]
        self.LaserScan_leftrow_avrg = np.average(self.LaserScan_leftrow)
        self.LaserScan_front_avrg = np.average(self.LaserScan_front)


    def pub_cmd(self, event=None):

        #Enkel P kontroller for kjoring langs midten av kjorefelt
        error_x = self.LaserScan_leftrow_avrg - 1.5
        speed_cmd = Twist();
        self.linearPaadrag = self.LaserScan_front_avrg - 2.5

        #if (self.linearPaadrag < 0.2) | ((self.LaserScan_leftrow_avrg == 3.0) & (self.LaserScan_rightrow_avrg == 3.0)):
        #    self.linearPaadrag = 0.0

        speed_cmd.linear.x = self.linearPaadrag
        speed_cmd.angular.z = error_x / 3;

        self.speed_pub.publish(speed_cmd)
        print(error_x)





def main():
    rospy.init_node('rowcentering_pointcl2lasersc_node', anonymous=True)
    rowcentering_pointcl2lasersc_objetct = RowCentering_pointcl2lasersc()

    #Lager en timer for aa kunne publisere til cmd ved et visst tidsintervall
    #dette for aa ikke faa lag paa kjoringen, da realsense/scan blir publiser
    #for sjeldent til aa unngaa lag
    rospy.Timer(rospy.Duration(1.0/10.0), rowcentering_pointcl2lasersc_objetct.pub_cmd)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__=='__main__':
    main()
