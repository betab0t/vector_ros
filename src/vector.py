#!/usr/bin/env python3.6

import rospy
import anki_vector

from vector_ros.srv import BatteryState, BatteryStateResponse

class VectorService:
    def __init__(self, robot):
        self.robot = robot
        self.battery_state_service = rospy.Service("~battery_state", BatteryState, self.battery_state_service_cb)

    def battery_state_service_cb(self, request):
        battery_state = self.robot.get_battery_state()
        response = BatteryStateResponse()
        response.battery_volts = battery_state.battery_volts
        response.battery_level = battery_state.battery_level
        response.is_charging = battery_state.is_charging
        response.is_on_charger_platform = battery_state.is_on_charger_platform
        response.suggested_charger_sec = battery_state.suggested_charger_sec

        return response

if __name__=="__main__":
    rospy.init_node("vector")
    robot = anki_vector.Robot()
    robot.connect()
    VectorService(robot)
    rospy.spin()
