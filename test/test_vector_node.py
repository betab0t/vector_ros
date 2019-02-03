#!/usr/bin/env python3.6

import rospy
import actionlib
import unittest

from std_msgs.msg import Int32

from vector_ros.srv import BatteryState
from vector_ros.msg import PlayAnimationAction, PlayAnimationGoal, PlayAnimationResult
from vector_ros.srv import AnimList, AnimListResponse
from vector_ros.srv import HeadAngle, HeadAngleResponse
from vector_ros.srv import LiftHeight, LiftHeightResponse

class TestVectorNode(unittest.TestCase):
    def test_battery_state_service(self):
        rospy.wait_for_service("/vector/battery_state")
        battery_state = rospy.ServiceProxy("/vector/battery_state", BatteryState)
        _battery_state = battery_state()

        self.assertTrue(1.0 <= _battery_state.battery_volts <= 5.0)

    def lwheel_ticks_cb(self, msg):
        self.lwheel_ticks = msg.data

    def rwheel_ticks_cb(self, msg):
        self.rwheel_ticks = msg.data

    def test_wheel_ticks_increase_while_driving(self):
        self.lwheel_ticks = self.rwheel_ticks = 0

        # subscribe to both wheel ticks topics
        rospy.Subscriber("/vector/lwheel_ticks", Int32, self.lwheel_ticks_cb)
        rospy.Subscriber("/vector/rwheel_ticks", Int32, self.rwheel_ticks_cb)

        lwheel_desired_rate_pubisher = rospy.Publisher("/vector/lwheel_desired_rate", Int32, queue_size=1)
        rwheel_desired_rate_pubisher = rospy.Publisher("/vector/rwheel_desired_rate", Int32, queue_size=1)

        _rate = rospy.Rate(10)

        # publish desired speed for both wheels
        for i in range(10):
            lwheel_desired_rate_pubisher.publish(data=int(100))
            rwheel_desired_rate_pubisher.publish(data=int(100))

            _rate.sleep()

        # check both wheel ticks increased
        self.assertTrue(self.lwheel_ticks > 0)
        self.assertTrue(self.rwheel_ticks > 0)

    def test_play_animation_action(self):
        play_animation = actionlib.SimpleActionClient("/vector/play_animation", PlayAnimationAction)
        play_animation.wait_for_server()
        goal = PlayAnimationGoal()
        goal.anim = "test_anim_01"
        play_animation.send_goal(goal)
        play_animation.wait_for_result()
        self.assertIsInstance(play_animation.get_result(), PlayAnimationResult)

    def test_anim_list_service(self):
        rospy.wait_for_service("/vector/anim_list")
        anim_list = rospy.ServiceProxy("/vector/anim_list", AnimList)
        res = anim_list()
        self.assertIn("anim_turn_left_01", res.anim_names)

    def test_set_head_angle_service(self):
        rospy.wait_for_service("/vector/set_head_angle")
        set_head_angle = rospy.ServiceProxy("/vector/set_head_angle", HeadAngle)
        res = set_head_angle(deg=float(45.0))
        self.assertIsInstance(res, HeadAngleResponse)

    def test_set_lift_height_service(self):
        rospy.wait_for_service("/vector/set_lift_height")
        set_lift_height = rospy.ServiceProxy("/vector/set_lift_height", LiftHeight)
        res = set_lift_height(height=(0.75))
        self.assertIsInstance(res, LiftHeightResponse)

if __name__=="__main__":
    import rostest
    rospy.init_node("tests_node")
    rostest.rosrun("vector_ros", "test_vector_node", TestVectorNode)