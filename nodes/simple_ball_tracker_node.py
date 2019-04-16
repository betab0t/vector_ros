#!/usr/bin/env python2.7

import rospy
import cv2
import cv_bridge
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

try:
    from vector_ros.srv import HeadAngle
    from vector_ros.srv import SayText
except ImportError:
    print("missing service message definitions! did you `catkin_make` and `source`?")


class SimpleBallTracker(object):
    def __init__(self, is_simulation=False):
        self.is_simulation = is_simulation
        self.is_ball_hidden = True

        self._set_head_horizontal()
        self._init_cmd_vel_publisher()
        self._init_speech()
        self._init_camera_feed()

    def _init_cmd_vel_publisher(self):
        self.cmd_vel_pubisher = rospy.Publisher("/vector/cmd_vel", Twist, queue_size=1)
        self.cmd_vel_msg = Twist()

    def _init_speech(self):
        if not self.is_simulation:
            # speech supported only in real robot
            rospy.wait_for_service("/vector/say_text")
            self.say_text = rospy.ServiceProxy("/vector/say_text", SayText)

    def _init_camera_feed(self):
        self.cv_bridge = cv_bridge.CvBridge()
        self.image_subscriber = rospy.Subscriber("/vector/camera", Image, self.image_callback)

    def _set_head_horizontal(self):
        if self.is_simulation:
            head_angle_publisher = rospy.Publisher("/vector/head_angle/command", Float64, queue_size=1)
            head_angle_publisher.publish(Float64(data=0.5))
        else:
            rospy.wait_for_service("/vector/set_head_angle")
            set_head_angle = rospy.ServiceProxy("/vector/set_head_angle", HeadAngle)
            set_head_angle(deg=float(0.0))

    def image_callback(self, img_msg):
        cv_image = self._get_cv2_image(img_msg)
        contours = self._get_red_objects_contours(cv_image)
        largest_area_red_object = self._get_largest_red_object(contours)
        self.is_ball_hidden = not self._is_valid_moments(largest_area_red_object)
        if self.is_ball_hidden:
            self._stop_robot()
            self._show_image(cv_image)
        else:
            red_ball_center = self._get_moments_center(largest_area_red_object)
            desired_z_axis_velocity = self._calc_desired_z_axis_velocity(red_ball_center, cv_image)
            self._rotate_robot(desired_z_axis_velocity)
            self._say_i_found_my_ball()
            self._show_image_with_marker(cv_image, red_ball_center)

    def _get_cv2_image(self, img_msg):
        return self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

    @staticmethod
    def _get_red_objects_contours(image):
        def filter_non_red_colors(rgb_image):
            hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
            mask_red_lower_range = cv2.inRange(hsv_image, np.array([0, 100, 100]), np.array([10, 255, 255]))
            mask_red_upper_range = cv2.inRange(hsv_image, np.array([160, 100, 100]), np.array([180, 255, 255]))
            return cv2.add(mask_red_lower_range, mask_red_upper_range)

        _, contours, _ = cv2.findContours(filter_non_red_colors(image), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    @staticmethod
    def _get_largest_red_object(contours):
        largest_area_object = None
        for contour in contours:
            moments = cv2.moments(contour)
            if largest_area_object is None or moments['m00'] > largest_area_object['m00']:
                largest_area_object = moments
        return largest_area_object

    def _say_text(self, text):
        if not self.is_simulation:
            self.say_text(text=text)
        rospy.loginfo(text)

    def _say_i_found_my_ball(self):
        if self.is_ball_hidden:
            self._say_text("I found my ball")
            self.is_ball_hidden = False

    @staticmethod
    def _get_moments_center(moments):
        return int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00'])

    @staticmethod
    def _is_valid_moments(moments):
        return moments and not any(moments[x] == 0 for x in ['m00', 'm10', 'm01'])

    @staticmethod
    def _calc_proportional_z_angle_velocity(image_width, red_ball_center_x, p=300):
        def is_ball_in_tolerance():
            return (image_width / 2 - (image_width / 12)) < red_ball_center_x < (image_width / 2 + (image_width / 12))

        return 0.0 if is_ball_in_tolerance() else -(red_ball_center_x - (image_width / 2)) / p

    def _rotate_robot(self, z_axis_velocity):
        self.cmd_vel_msg.angular.z = float(z_axis_velocity)
        self.cmd_vel_pubisher.publish(self.cmd_vel_msg)

    def _stop_robot(self):
        self._rotate_robot(0.0)

    def _calc_desired_z_axis_velocity(self, red_ball_center, image):
        _, width, _ = image.shape
        cx = red_ball_center[0]
        return self._calc_proportional_z_angle_velocity(width, cx)

    @staticmethod
    def _show_image(image):
        cv2.imshow("Vector View", image)
        cv2.waitKey(1)

    @staticmethod
    def _show_image_with_marker(image, marker_pos):
        cv2.circle(image, marker_pos, 10, (0, 255, 0), -1)
        SimpleBallTracker._show_image(image)


if __name__=="__main__":
    rospy.init_node("simple_ball_tracker")
    SimpleBallTracker(rospy.get_param("~is_simulation", False))
    rospy.spin()
