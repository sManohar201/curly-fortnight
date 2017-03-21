#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy as np
from collections import deque
import argparse
import sys
from geometry_msgs.msg import Twist


kernel_size = 5
low_threshold = 80
high_threshold = 120
# define range of blue color in HSV
lower_color = np.array([100, 100, 100])
high_color = np.array([155, 255, 255])

# Used to erode image
kernel = np.ones((5, 5), np.uint8)

# Trail behind object
pts = []

average_x = 0.5
average_distance = 1.0

inside_box = False
distance_box = False

goal_distance = 1.0

relative_x = 0.5
distance = goal_distance
speed = 0.0
rotation = 0.0

class Tracker:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("input", 1)
        cv2.namedWindow("processed", 1)
        self.image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        self.motion = Twist()

        rate = rospy.Rate(20)

        # publish to cmd_vel of the jackal
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        while not rospy.is_shutdown():
            # publish Twist
            pub.publish(self.motion)

            rate.sleep()

    def slow_to_stop():
        # Nothing found, slow to a stop
        global average_x
        global average_distance
        global goal_distance
        global relative_x
        global distance
        global speed
        global rotation

        relative_x = 0.5
        average_x = average_x * 0.7 + relative_x * 0.3
        distance = goal_distance
        average_distance = average_distance * 0.7 + distance * 0.3
        speed = (average_distance - goal_distance)
        rotation = (0.5 - average_x) * 3.0

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width, channels = img.shape
        proc = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        proc = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, high_color)
        proc = cv2.bitwise_and(proc, proc, mask = mask)
        proc = cv2.erode(proc, kernel, 10)
        proc = cv2.dilate(proc, kernel, 10)
        edge = cv2.Canny(proc, low_threshold, high_threshold)
        contours = cv2.findContours(edge.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

        global relative_x
        global distance
        global speed
        global rotation

        speed = 0.0
        rotation = 0.0

        if len(contours) > 0:
            biggest_shape = None
            largest_area = 20.0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > largest_area:
                    largest_area = area
                    biggest_shape = contour

            if biggest_shape is None:
                slow_to_stop()
                break

            # compute the center of the contour
            M = cv2.moments(biggest_shape)
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            # draw the contour and center of the shape on the image
            (x, y), radius = cv2.minEnclosingCircle(biggest_shape)
            center = (int(x), int(y))
            radius = int(radius)

            global average_x
            global average_distance

            distance = 36.0 / radius

            relative_x = float(center_x) / width
            average_x = average_x * 0.7 + relative_x * 0.3

            average_distance = average_distance * 0.7 + distance * 0.3

            cv2.circle(proc, center, radius, (0, 255, 0), 2)

            pts.append((center_x, center_y))
            if len(pts) > 20:
                pts.pop(0)

            global inside_box
            global distance_box

            if inside_box:
                if average_x > 0.6 or average_x < 0.4:
                    inside_box = False
                    rotation = (0.5 - average_x) * 3.0
                else:
                    rotation = 0.0
            else:
                if average_x < 0.55 and average_x > 0.45:
                    inside_box = True
                    rotation = 0.0
                else:
                    rotation = (0.5 - average_x) * 3.0

            if distance_box:
                if average_distance > goal_distance + 0.2 or average_distance < goal_distance - 0.2:
                    distance_box = False
                    speed = (average_distance - goal_distance) * 0.5
                else:
                    speed = 0.0
            else:
                if average_distance < goal_distance + 0.1 and average_distance > goal_distance - 0.1:
                    distance_box = True
                    speed = 0.0
                else:
                    speed = (average_distance - goal_distance) * 0.5

            if speed > 0.75:
                speed = 0.75
            elif speed < -0.75:
                speed = -0.75

            if rotation > 0.75:
                rotation = 0.75
            elif rotation < -0.75:
                rotation = -0.75

            if inside_box:
                cv2.rectangle(proc, (int(width * 0.6), 0), (int(width * 0.4), height), (0, 255, 0), 3)
            else:
                cv2.rectangle(proc, (int(width * 0.55), 0), (int(width * 0.45), height), (0, 0, 255), 3)

            cv2.circle(proc, (center_x, center_y), 7, (255, 255, 255), -1)
            cv2.circle(proc, (int(average_x * width), center_y), 7, (0, 255, 0), -1)
            cv2.putText(proc, "center", (center_x - 20, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # loop over the set of tracked points
            for i in xrange(1, len(pts)):
                # if either of the tracked points are None, ignore them
                if pts[i - 1] is None or pts[i] is None:
                    continue

                # otherwise, compute the thickness of the line and
                # draw the connecting lines
                thickness = int(np.sqrt(float(i + 1) / 1.0) * 2.5)
                cv2.line(proc, pts[i - 1], pts[i], (0, 0, 255), thickness)
        else:
            slow_to_stop()

        print("speed = {}\trotation = {}\taverage distance = {}".format(speed, rotation, average_distance))

         # move forward
        self.motion.linear.x  = speed
        self.motion.angular.z = rotation

        cv2.imshow("input", img)
        cv2.imshow("processed", proc)
        cv2.moveWindow("processed", 850, 0)
        cv2.waitKey(3)

rospy.init_node('tracker')
tracker_proto = Tracker()
rospy.spin()
