#!/usr/bin/env python2.7

import rospy
import cv2
import cv_bridge
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

try:
    from vector_ros.srv import HeadAngle
    from vector_ros.srv import SayText
except ImportError:
    print("missing service message definitions! did you `catkin_make` and `source` vector_ros package/ws?")

class SimpleBallTracker(object):
    def __init__(self):
        self.set_head_horizontal()
        self.cmd_vel_pubisher = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)
        self.cmd_vel_msg = Twist()

        # init camera feed
        self.cv_bridge = cv_bridge.CvBridge()
        self.image_subscriber = rospy.Subscriber("/vector/camera", Image, self.image_cb)

        # hsv color ranges of our red ball
        self.hsv_color_ranges = ((np.array([0, 70, 50]), np.array([10, 255, 255])), (np.array([170, 70, 50]), np.array([180, 255, 255])))

        rospy.wait_for_service("/vector/say_text")
        self.say_text = rospy.ServiceProxy("/vector/say_text", SayText)
        self.is_ball_hidden = True

    def set_head_horizontal(self):
        rospy.wait_for_service("/vector/set_head_angle")
        set_head_angle = rospy.ServiceProxy("/vector/set_head_angle", HeadAngle)
        set_head_angle(deg=float(0.0))

    def image_cb(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width, _ = cv_image.shape
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # red has two hsv ranges, create a mask for both & combine them
        mask_red_lower_range = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        mask_red_upper_range = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
        mask = cv2.add(mask_red_lower_range, mask_red_upper_range)

        # find largest-area contour
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        largest_area_object = None
        for contour in contours:
            moments = cv2.moments(contour)
            if largest_area_object is None or moments['m00'] > largest_area_object['m00']:
                largest_area_object = moments

        # find center of contour
        if largest_area_object and not any(largest_area_object[x] == 0 for x in ['m00', 'm10', 'm01']):
            ball_center = cx, cy = int(largest_area_object['m10'] / largest_area_object['m00']), int(largest_area_object['m01'] / largest_area_object['m00'])
            cv2.circle(cv_image, ball_center, 10, (0, 255, 0), -1)

            # say something if the ball was previously hidden
            if self.is_ball_hidden == True:
                self.say_text(text="I found my ball")
                self.is_ball_hidden = False

            # check if the ball is approximately centered, else apply simple proportional command
            if (width / 2 - (width / 12)) < cx < (width / 2 + (width / 12)):
                self.cmd_vel_msg.angular.z = 0.0

            else:
                self.cmd_vel_msg.angular.z = -(cx - (width / 2)) / 300 # P=300

        else:
            self.cmd_vel_msg.angular.z = 0.0
            self.is_ball_hidden = True

        # move vector
        self.cmd_vel_pubisher.publish(self.cmd_vel_msg)

        cv2.imshow("Vector View", cv_image)
        cv2.waitKey(1)

if __name__=="__main__":
    rospy.init_node("simple_ball_tracker")
    SimpleBallTracker()
    rospy.spin()