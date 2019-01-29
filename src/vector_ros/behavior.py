#!/usr/bin/env python3.6

import rospy
import anki_vector

from vector_ros.srv import HeadAngle, HeadAngleResponse, LiftHeight, LiftHeightResponse

class Behavior(object):
    '''Expose functions list at https://developer.anki.com/vector/docs/generated/anki_vector.behavior.html'''

    def __init__(self, robot):
        self.robot = robot
        self.set_head_angle_service = rospy.Service("~set_head_angle", HeadAngle, self.set_head_angle_service_cb)
        self.set_lift_height_service=rospy.Service("~set_lift_height", LiftHeight, self.set_lift_height_service_cb)

    def set_head_angle_service_cb(self, request):
        self.robot.behavior.set_head_angle(anki_vector.util.degrees(request.deg))
        return HeadAngleResponse()

    def set_lift_height_service_cb(self, request):
        self.robot.behavior.set_lift_height(request.height)
        return LiftHeightResponse()

if __name__=="__main__":
    rospy.init_node("vector")
    robot = anki_vector.Robot()
    robot.connect()
    Behavior(robot)
    rospy.spin()
