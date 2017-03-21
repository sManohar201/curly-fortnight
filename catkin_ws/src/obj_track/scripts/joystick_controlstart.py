#!/usr/bin/env python

# joystick_controlstart.py
# Use joystick input to launch exploration nodes in jackal
# Intro to Robotics - EE5900 - Spring 2017
#          Assignment #6

#       Project #6 Group #4
#            Haden
#            Phillip
#            Sabari (Teamlead)
#
# /blueooth_teleop/joy
# sensor_msgs/Joy
#
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# float32[] axes
# int32[] buttons
#
# axes: [ lft - l/r, lft - up/down, L2 (1/-1), rgt - l/r, rgt - u/d, R2 (1/-1)]
# buttons: [ x, circle, sq, tri, L1, R1, share, options, play, L3, R3, DL, DR, DU, DD]
#

import rospy
import roslaunch
import sys
import time
import os
import objtrack
from sensor_msgs.msg import Joy

class joy_control(object):

    def __init__(self):

        rate = rospy.Rate(5)
        rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.joy_callback)

        self.x = 0
        self.circ = 0
        self.sq = self.tri = self.L1 = self.R1 = self.share = self.options = self.p4 = self.L3 = self.R3 = self.DL = self.DR = self.DU = self.DD = 0

        self.active = False
        objtrack_process = None

        rospy.loginfo("In start")

        while not rospy.is_shutdown():


            if self.active == False:

                # Start Object Tracking
                if (self.circ == 1):
                    rospy.loginfo("Joystick code received, commencing object tracking protocol...")
                    self.active = True

                    package = 'obj_track'
                    executable = 'track.py' # NEED TO UPDATE ACTUAL SCRIPT NAME

                    node = roslaunch.core.Node(package, executable)
                    launch = roslaunch.scriptapi.ROSLaunch()
                    launch.start()
                    objtrack_process = launch.launch(node)
            else:

                # Stop Object Tracking
                if (self.x == 1):
                    rospy.loginfo("Joystick code recieved, terminating object tracking protocol")
                    self.active = False
                    objtrack_process.stop()

            self.x = 0
            self.circ = 0
            #rate.sleep()

    def joy_callback(self, data):

        self.x, self.circ, self.sq, self.tri, self.L1, self.R1, self.share, self.options, self.p4, self.L3, self.R3, self.DL, self.DR, self.DU, self.DD = data.buttons


if __name__ == "__main__":

    try:
        rospy.init_node("joy_start", anonymous=False)
        run = joy_control()  #read in joystick input
    except rospy.ROSInterruptException:
        rospy.loginfo("joy_start node terminated.")
