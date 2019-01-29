#!/usr/bin/env python3.6

import rospy
import anki_vector

from std_msgs.msg import Int32


class Drive(object):
    def __init__(self, async_robot, publish_rate=2):
        self.async_robot = async_robot
        self.publish_rate = publish_rate
        self.rate = rospy.Rate(self.publish_rate)

        # publishers
        self.lwheel_ticks_publisher = rospy.Publisher("~lwheel_ticks", Int32, queue_size=1)
        self.rwheel_ticks_publisher = rospy.Publisher("~rwheel_ticks", Int32, queue_size=1)

        # subscribers
        self.lwheel_desired_rate_subscriber = rospy.Subscriber("~lwheel_desired_rate", Int32, self.lwheel_desired_rate_cb)
        self.rwheel_desired_rate_subscriber = rospy.Subscriber("~rwheel_desired_rate", Int32, self.rwheel_desired_rate_cb)

        # wheels speed
        self.lwheel_speed_mm_sec = 0
        self.rwheel_speed_mm_sec = 0

        self.publish_estimated_encoder_ticks()

    # callbacks
    def lwheel_desired_rate_cb(self, msg):
        self.lwheel_speed_mm_sec = msg.data
        self.async_robot.motors.set_wheel_motors(self.lwheel_speed_mm_sec, self.rwheel_speed_mm_sec)

    def rwheel_desired_rate_cb(self, msg):
        self.rwheel_speed_mm_sec = msg.data
        self.async_robot.motors.set_wheel_motors(self.lwheel_speed_mm_sec, self.rwheel_speed_mm_sec)

    def publish_estimated_encoder_ticks(self):
        ''' publishing encoder ticks is required in order to use diff_drive pkg '''

        lwheel_ticks_total = 0
        rwheel_ticks_total = 0

        while not rospy.is_shutdown():
            if self.async_robot.left_wheel_speed_mmps > 0:
                lwheel_ticks_total += self.async_robot.left_wheel_speed_mmps / self.publish_rate

            if self.async_robot.right_wheel_speed_mmps > 0:
                rwheel_ticks_total += self.async_robot.right_wheel_speed_mmps / self.publish_rate

            # publish number of estimated ticks for each wheel in ratio of 1000 ticks per meter
            self.lwheel_ticks_publisher.publish(data=int(lwheel_ticks_total))
            self.rwheel_ticks_publisher.publish(data=int(rwheel_ticks_total))

            self.rate.sleep() # publish at specified rate

if __name__=="__main__":
    rospy.init_node("drive")
    async_robot = anki_vector.AsyncRobot()
    async_robot.connect()
    Drive(async_robot)
    rospy.spin()