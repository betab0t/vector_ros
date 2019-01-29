#!/usr/bin/env python3.6

import rospy
import anki_vector
import threading

from vector_ros.vector import Vector
from vector_ros.anim import Animation
from vector_ros.drive import Drive
from vector_ros.camera import Camera

if __name__=="__main__":
    rospy.init_node("vector")

    # connect to Vector
    async_robot = anki_vector.AsyncRobot(enable_camera_feed=True)
    async_robot.connect()

    # start all using shared AsyncRobot object
    Vector(async_robot)
    Animation(async_robot)

    # these services require threads to run(and publish) in parallel
    drive_thread = threading.Thread(target=Drive, args=(async_robot,))
    drive_thread.start()

    camera_thread = threading.Thread(target=Camera, args=(async_robot,))
    camera_thread.start()

    rospy.spin()