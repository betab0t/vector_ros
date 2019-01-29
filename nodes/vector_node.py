#!/usr/bin/env python3.6

import rospy
import anki_vector

from vector_ros.vector import Vector
from vector_ros.anim import Animation
from vector_ros.drive import Drive


if __name__=="__main__":
    rospy.init_node("vector")

    # connect to Vector
    async_robot = anki_vector.AsyncRobot()
    async_robot.connect()

    # start all using shared AsyncRobot object
    Vector(async_robot)
    Animation(async_robot)
    Drive(async_robot)

    rospy.spin()