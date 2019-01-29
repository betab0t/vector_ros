#!/usr/bin/env python3.6

import rospy
import actionlib

import anki_vector

from vector_ros.msg import PlayAnimationAction

class Animation:
    def __init__(self, robot):
        self.robot = robot
        self.rate = rospy.Rate(4)
        self.action_server = actionlib.SimpleActionServer("~/play_animation", PlayAnimationAction, self.play_animation_cb, False)
        self.action_server.start()

    def play_animation_cb(self, msg):
        job = self.robot.anim.play_animation(msg.anim)
        while job.running():
            if self.action_server.is_preempt_requested():
                job.cancle()
                self.action_server.set_preempt()

            self.rate.sleep()

        if not job.cancelled():
            self.action_server.set_succeeded()


if __name__=="__main__":
    rospy.init_node("vector_anim")
    robot = anki_vector.AsyncRobot()
    robot.connect()
    animation = Animation(robot)
    rospy.spin()