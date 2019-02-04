#!/usr/bin/env python3.6

import rospy
import anki_vector
import concurrent.futures

from vector_ros.srv import BatteryState, BatteryStateResponse

class Vector(object):
    '''Expose functions list at https://developer.anki.com/vector/docs/generated/anki_vector.html'''

    def __init__(self, robot):
        self.robot = robot
        self.battery_state_service = rospy.Service("~battery_state", BatteryState, self.battery_state_service_cb)

    def battery_state_service_cb(self, request):
        job = self.robot.get_battery_state()
        response = BatteryStateResponse()

        try:
            battery_state = job.result(2) # wait max 2 secs
        except concurrent.futures.TimeoutError:
            rospy.logerr("timeout - could not get battery state!")
            return response

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
    Vector(robot)
    rospy.spin()
