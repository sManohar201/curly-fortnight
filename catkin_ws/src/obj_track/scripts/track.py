#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy as np
from collections import deque
import argparse
import sys


kernel_size = 5
low_threshold = 80
high_threshold = 120
# define range of blue color in HSV
lower_color = np.array([20, 100, 50])
high_color = np.array([70, 255, 255])

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

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width, channels = img.shape
        proc = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        proc = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, high_color)
        proc = cv2.bitwise_and(proc, proc, mask = mask)
        proc = cv2.erode(proc, kernel, 20)
        proc = cv2.dilate(proc, kernel, 2)
        edge = cv2.Canny(proc, low_threshold, high_threshold)
        contours = cv2.findContours(edge.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

        if contours is not None:
        # len(contours) > 0:
            biggest_shape = None
            most_edges = 20
            for contour in contours:
                if len(contour) > most_edges:
                    most_edges = len(contour)
                    biggest_shape = [contour]

            if biggest_shape is not None:
                # compute the center of the contour
                #M = cv2.moments(biggest_shape[0])
                #if M["m00"] > 0.0001:
                #cX = int(M["m10"] / M["m00"])
                #cY = int(M["m01"] / M["m00"])
                cX = int(width / 2.0)
                cY = int(height / 2.0)
                relativeX = float(cX) / width
                global averageX
                averageX = 0.8 * averageX + 0.2 * relativeX
                print(relativeX)
                print(averageX)
                pts.append((int(averageX * width), cY))
                if len(pts) > 20:
                    pts.pop(0)

                # draw the contour and center of the shape on the image
                epsilon = 0.02 * cv2.arcLength(biggest_shape[0], True)
                approx = cv2.approxPolyDP(biggest_shape[0], epsilon, True)

                #rect = cv2.minAreaRect(biggest_shape[0])
                #box = cv2.boxPoints(rect)
                #box = np.int0(box)

                cv2.drawContours(proc, [approx], -1, (255, 0, 0), 3)
                if inside_box:
                    if averageX > 0.6 or averageX < 0.4:
                        inside_box = False
                else:
                    if averageX < 0.55 and averageX > 0.45:
                        inside_box = True
                if inside_box:
                    cv2.rectangle(proc, (int(width * 0.6), 0), (int(width * 0.4), height), (0, 255, 0), 3)
                else:
                    cv2.rectangle(proc, (int(width * 0.55), 0), (int(width * 0.45), height), (0, 0, 255), 3)
                cv2.circle(proc, (cX, cY), 7, (255, 255, 255), -1)
                cv2.circle(proc, (int(averageX * width), cY), 7, (0, 255, 0), -1)
                cv2.putText(proc, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # loop over the set of tracked points
                for i in xrange(1, len(pts)):
                    # if either of the tracked points are None, ignore them
                    if pts[i - 1] is None or pts[i] is None:
                        continue

                    # otherwise, compute the thickness of the line and
                    # draw the connecting lines
                    thickness = int(np.sqrt(float(i + 1) / 1.0) * 2.5)
                    cv2.line(proc, pts[i - 1], pts[i], (0, 0, 255), thickness)

        cv2.imshow("input", img)
        cv2.imshow("processed", proc)
        cv2.moveWindow("processed", 850, 0)
        cv2.waitKey(3)

rospy.init_node('tracker')
tracker_proto = Tracker()
rospy.spin()
