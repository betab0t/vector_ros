#!/usr/bin/env python3.6

import rospy
import anki_vector

import vector
import anim
import drive


if __name__=="__main__":
    rospy.init_node("vector")

    # connect to Vector
    async_robot = anki_vector.AsyncRobot()
    async_robot.connect()

    # start all using shared AsyncRobot object
    vector.Vector(async_robot)
    anim.Animation(async_robot)
    drive.Drive(async_robot)

    rospy.spin()