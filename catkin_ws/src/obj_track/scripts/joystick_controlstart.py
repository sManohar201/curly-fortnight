#!/usr/bin/env python

# joy_trigger_start.py
# Use joystick input to launch exploration nodes in jackal
# Intro to Robotics - EE5900 - Spring 2017
#          Assignment #5

#       Project #6 Group #4
#         Haden
#            Phillip
#            Sabari
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
import wall_avoid
import jackal_move
from sensor_msgs.msg import Joy

class joy_control(object):

    def __init__(self):

        rate = rospy.Rate(5)
        rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.joy_callback)

        self.x, self.circ = 0
        self.sq, self.tri, self.L1, self.R1, self.share, self.options, self.p4, self.L3, self.R3, self.DL, self.DR, self.DU, self.DD = 0
        #self.llr, self.lud, self.L2, self.rlr, self.rud, self.R2 = 0

        self.active = False
        #self.trigger = False
        #self.explore_mode = 0
        objtrack_process = None
        #timer_start = 0
        rospy.loginfo("In start")

        while not rospy.is_shutdown():
            #if (self.trigger == True):

            if self.active == False:

                # Start Object Tracking
                if (self.x == 1):
                    rospy.loginfo("Joystick code received, commencing object tracking protocol...")
                    self.active = True
                    # run = wall_avoid.WallAvoid(runtime)
                    package = 'obj_track'
                    executable = 'obj_track.py' # NEED TO UPDATE ACTUAL SCRIPT NAME

                    node = roslaunch.core.Node(package, executable)
                    launch = roslaunch.scriptapi.ROSLaunch()
                    launch.start()
                    objtrack_process = launch.launch(node)
            else:

                # Stop Object Tracking
                if (self.circ == 1):
                    rospy.loginfo("Joystick code recieved, terminating object tracking protocol")
                    self.active = False
                    objtrack_process.stop()
            #self.trigger = False
            self.x = 0
            self.circ = 0
            #rate.sleep()

    def joy_callback(self, data):

        self.x, self.circ, self.sq, self.tri, self.L1, self.R1, self.share, self.options, self.p4, self.L3, self.R3, self.DL, self.DR, self.DU, self.DD = data.buttons
        #llr, lud, L2, rlr, rud, R2 = data.axes

        # # Start Object Tracking
        # if (x == 1) and (L2 < 0.9) and (R2 < 0.9) and (self.ready == False):
        #     rospy.loginfo("Controller code received, commencing exploration protocol...")
        #     self.trigger = True
        #     self.ready = True
        # # Stop Object Tracking
        # if (tri == 1) and (L2 < 0.9) and (R2 < 0.9) and (self.ready == True):
        #     rospy.loginfo("Controller code received, terminating exploration protocol...")
        #     self.trigger = True
        #     self.ready = False
        # rospy.sleep(1)


if __name__ == "__main__":

    try:
        rospy.init_node("joy_start", anonymous=False)
        run = joy_control()  #read in joystick input
    except rospy.ROSInterruptException:
        rospy.loginfo("joy_start node terminated.")
