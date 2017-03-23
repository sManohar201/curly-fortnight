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

# Trail behind object
# pts = []
#
# average_x = 0.5
# average_distance = 1.0
#
# inside_box = False
# distance_box = False
#
# goal_distance = 1.0
#
# relative_x = 0.5
# distance = goal_distance
# speed = 0.0
# rotation = 0.0
#
# previous_speed = 0.0
# previous_rotation = 0.0
#
# distance_set = 0
#
# def nothing(x):
#     pass

# Human face tracker
# Most code obtained from track.py and adapted for this script
# Currently unfinished

class Tracker:
    def __init__(self):
        # Pre-trained classifiers, obtained from https://github.com/opencv/opencv/tree/master/data/haarcascades
        self.face_cascade = cv2.CascadeClassifier('/home/robotics/git_work/curly-fortnight/catkin_ws/src/obj_track/scripts/haarcascade_frontalface_default.xml')
        self.eye_cascade = cv2.CascadeClassifier('/home/robotics/git_work/curly-fortnight/catkin_ws/src/obj_track/scripts/haarcascade_eye.xml')

        self.bridge = cv_bridge.CvBridge()

        cv2.namedWindow("window1", 1)
        self.image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        self.motion = Twist()

        rate = rospy.Rate(20)

        # publish to cmd_vel of the jackal
        pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=10)

        while not rospy.is_shutdown():
            # publish Twist
            pub.publish(self.motion)
            rate.sleep()

    # Reused code from track.py
    def slow_to_stop(self):
        # Nothing found, slow to a stop
        global average_x
        global average_distance
        global goal_distance
        global relative_x
        global distance
        global speed
        global rotation

        relative_x = 0.5
        average_x = average_x * 0.8 + relative_x * 0.2
        distance = goal_distance
        average_distance = average_distance * 0.8 + distance * 0.2
        speed = (average_distance - goal_distance)
        rotation = (0.5 - average_x) * 3.0


    def image_callback(self, msg):
        global relative_x
        global distance
        global speed
        global rotation
        global goal_distance

        speed = 0.0
        rotation = 0.0
        # Detect face using Pre-trained classifier
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        height, width, channels = img.shape
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 3)
        print faces
        for (x,y,w,h) in faces:
            rect_x = x
            rect_y = y
            rect_w = w
            rect_h = h
            print "x: %d, y: %d, w:, %d, h:, %d" % (x, y, w, h)
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            eyes = self.eye_cascade.detectMultiScale(roi_gray)
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

        # output = img.copy()

        if len(faces) != 0:
            print "Face Detected"

            # draw the contour and center of the shape on the image
            # center = rect_x + rect_w + rect_h
            #
            # global average_x
            # global average_distance
            #
            # distance = 36.0 / rect_w
            #
            # relative_x = float(center) / rect_w
            # average_x = average_x * 0.3 + relative_x * 0.7
            #
            # average_distance = average_distance * 0.7 + distance * 0.3
            #
            # global distance_set
            #
            # if distance_set < 11:
            #     if distance_set == 10:
            #         goal_distance = average_distance
            #     distance_set += 1
            # else:
            #     # pts.append((int(average_x * width), center_y))
            #     if len(pts) > 20:
            #         pts.pop(0)
            #
            #     global inside_box
            #     global distance_box
            #
            #     if inside_box:
            #         if average_x > 0.6 or average_x < 0.4:
            #             inside_box = False
            #             rotation = (0.5 - average_x) * 2.0
            #         else:
            #             rotation = 0.0
            #     else:
            #         if average_x < 0.55 and average_x > 0.45:
            #             inside_box = True
            #             rotation = 0.0
            #         else:
            #             rotation = (0.5 - average_x) * 2.0
            #
            #     if distance_box:
            #         if average_distance > goal_distance + 0.2 or average_distance < goal_distance - 0.2:
            #             distance_box = False
            #             speed = (average_distance - goal_distance) * 0.4
            #         else:
            #             speed = 0.0
            #     else:
            #         if average_distance < goal_distance + 0.1 and average_distance > goal_distance - 0.1:
            #             distance_box = True
            #             speed = 0.0
            #         else:
            #             speed = (average_distance - goal_distance) * 0.4
            #
            #     if speed > 0.75:
            #         speed = 0.75
            #     elif speed < -0.75:
            #         speed = -0.75
            #
            #     if rotation > 0.75:
            #         rotation = 0.75
            #     elif rotation < -0.75:
            #         rotation = -0.75
            #
            #     if inside_box:
            #         cv2.rectangle(gray, (int(width * 0.6), 0), (int(width * 0.4), height), (0, 255, 0), 3)
            #     else:
            #         cv2.rectangle(gray, (int(width * 0.55), 0), (int(width * 0.45), height), (0, 0, 255), 3)

                # # loop over the set of tracked points
                # for i in xrange(1, len(pts)):
                #     # if either of the tracked points are None, ignore them
                #     if pts[i - 1] is None or pts[i] is None:
                #         continue
                #
                #     # otherwise, compute the thickness of the line and
                #     # draw the connecting lines
                #     thickness = int(np.sqrt(float(i + 1) / 1.0) * 2.5)
                #     cv2.line(gray, pts[i - 1], pts[i], (0, 0, 255), thickness)


        else:
            print "No face detected"
            # self.slow_to_stop()

        # print("speed = {}\trotation = {}\taverage distance = {}".format(speed, rotation, average_distance))
        # print(goal_distance)
        #
        # global previous_speed
        # global previous_rotation
        #
        # acceleration_limit = 0.02
        # rotation_limit = 0.02
        #
        # if speed > previous_speed + acceleration_limit:
        #     speed = previous_speed + acceleration_limit
        #     previous_speed = speed
        # elif speed < previous_speed - acceleration_limit:
        #     speed = previous_speed - acceleration_limit
        #     previous_speed = speed
        #
        # if rotation > previous_rotation + rotation_limit:
        #     rotation = previous_rotation + rotation_limit
        #     previous_rotation = rotation
        # elif rotation < previous_rotation - rotation_limit:
        #     rotation = previous_rotation - rotation_limit
        #     previous_rotation = rotation

         # move forward
        # self.motion.linear.x = speed
        # self.motion.angular.z = rotation

        cv2.imshow("window1", img)
        # cv2.imshow("window2", gray)
        cv2.waitKey(3)

rospy.init_node('Track_Marker')
Track_Marker = Tracker()
rospy.spin()
