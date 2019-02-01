#!/usr/bin/env python3.6

import rospy
import unittest

from vector_ros.srv import BatteryState

class TestVectorNode(unittest.TestCase):
    def test_battery_state_service(self):
        rospy.wait_for_service("/vector/battery_state")
        battery_state = rospy.ServiceProxy("/vector/battery_state", BatteryState)
        _battery_state = battery_state()

        self.assertTrue(1.0 <= _battery_state.battery_volts <= 5.0)

if __name__=="__main__":
    import rostest
    rostest.rosrun("vector_ros", "test_vector_node", TestVectorNode)