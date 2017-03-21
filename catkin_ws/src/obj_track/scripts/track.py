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

averageX = 0.5

inside_box = False

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

        speed = 0.0
        rotation = 0.0

        if len(contours) > 0:
            biggest_shape = contours[0]
            largest_area = 0.0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > largest_area:
                    largest_area = area
                    biggest_shape = contour

            # compute the center of the contour
            M = cv2.moments(biggest_shape)
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            print("X = {}\tY = {}".format(center_x, center_y))
            # draw the contour and center of the shape on the image
            (x, y), radius = cv2.minEnclosingCircle(biggest_shape)
            center = (int(x), int(y))
            radius = int(radius)

            distance = 36.0 / radius
            print('{} m\t{} pixels'.format(distance, radius))
            cv2.circle(proc, center, radius, (0, 255, 0), 2)

            global averageX

            relativeX = float(center_x) / width
            averageX = averageX * 0.7 + relativeX * 0.3

            pts.append((center_x, center_y))
            if len(pts) > 20:
                pts.pop(0)

            global inside_box

            if inside_box:
                if averageX > 0.6 or averageX < 0.4:
                    inside_box = False
                    rotation = (averageX - 0.5) * 0.1
                else:
                    rotation = 0.0
            else:
                if averageX < 0.55 and averageX > 0.45:
                    inside_box = True
                    rotation = 0.0
                else:
                    rotation = (averageX - 0.5) * 0.1
            if inside_box:
                cv2.rectangle(proc, (int(width * 0.6), 0), (int(width * 0.4), height), (0, 255, 0), 3)
            else:
                cv2.rectangle(proc, (int(width * 0.55), 0), (int(width * 0.45), height), (0, 0, 255), 3)

            cv2.circle(proc, (center_x, center_y), 7, (255, 255, 255), -1)
            cv2.circle(proc, (int(averageX * width), center_y), 7, (0, 255, 0), -1)
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
            # Nothing found, sit still
            speed = 0.0
            rotation = 0.0
            pass

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
