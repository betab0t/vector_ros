#!/usr/bin/env python3.6

import rospy
import anki_vector

from sensor_msgs.msg import JointState

class JointStatesPublisher(object):
    def __init__(self, async_robot, publish_rate=10):
        self.async_robot = async_robot
        self.rate = rospy.Rate(publish_rate)
        self.joint_states_publisher = rospy.Publisher("~joint_states", JointState, queue_size=1)
        self.publish_joint_states()

    def publish_joint_states(self):
        joint_states = JointState()
        joint_states.name = ["base_to_head"]
        while not rospy.is_shutdown():
            joint_states.position = [-1 * self.async_robot.head_angle_rad]
            self.joint_states_publisher.publish(joint_states)
            self.rate.sleep()

if __name__=="__main__":
    rospy.init_node("tf")
    async_robot = anki_vector.AsyncRobot()
    async_robot.connect()
    JointStatesPublisher(async_robot)
    rospy.spin()

