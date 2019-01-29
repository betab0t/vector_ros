#!/usr/bin/env python3.6

import rospy
import anki_vector
import cv_bridge
import numpy

from sensor_msgs.msg import Image

class Camera(object):
    def __init__(self, async_robot, publish_rate=10):
        self.async_robot = async_robot
        self.rate = rospy.Rate(publish_rate)
        self.image_publisher = rospy.Publisher("~camera", Image, queue_size=1)
        self.publish_camera_feed()

    def publish_camera_feed(self):
        bridge=cv_bridge.CvBridge()

        while not rospy.is_shutdown():
            image = bridge.cv2_to_imgmsg(numpy.array(self.async_robot.camera.latest_image)) # convert PIL.Image to ROS Image
            self.image_publisher.publish(image)

            self.rate.sleep()

if __name__=="__main__":
    rospy.init_node("camera")
    async_robot = anki_vector.AsyncRobot(enable_camera_feed=True)
    async_robot.connect()
    Camera(async_robot)
    rospy.spin()

