#!/usr/bin/env python3.6

import rospy
import actionlib

import anki_vector

from vector_ros.msg import PlayAnimationAction
from vector_ros.srv import AnimList, AnimListResponse

class Animation:
    '''Expose functions list at https://developer.anki.com/vector/docs/generated/anki_vector.animation.html'''

    def __init__(self, robot):
        self.robot = robot
        self.rate = rospy.Rate(4)

        self.action_server = actionlib.SimpleActionServer("~play_animation", PlayAnimationAction, self.play_animation_cb, False)
        self.action_server.start()

        self.anim_list_service = rospy.Service("~anim_list", AnimList, self.anim_list_cb)

    def play_animation_cb(self, msg):
        job = self.robot.anim.play_animation(msg.anim)
        while job.running():
            if self.action_server.is_preempt_requested():
                job.cancle()
                self.action_server.set_preempt()

            self.rate.sleep()

        if not job.cancelled():
            self.action_server.set_succeeded()

    def anim_list_cb(self, request):
        response = AnimListResponse()
        response.anim_names = self.robot.anim.anim_list

        return response


if __name__=="__main__":
    rospy.init_node("vector_anim")
    robot = anki_vector.AsyncRobot()
    robot.connect()
    animation = Animation(robot)
    rospy.spin()