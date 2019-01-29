#!/usr/bin/env python3.6

import rospy
import anki_vector

import vector
import anim

from std_msgs.msg import Int32

rospy.init_node("vector")

RATE = 2
rate = rospy.Rate(RATE)

robot = anki_vector.AsyncRobot()
robot.connect()

# publishers
lwheel_ticks_publisher = rospy.Publisher("~lwheel_ticks", Int32, queue_size=1)
rwheel_ticks_publisher = rospy.Publisher("~lwheel_ticks", Int32, queue_size=1)

# speeds
lwheel_speed_mm_sec = 0
rwheel_speed_mm_sec = 0

# callbacks
def lwheel_desired_rate_cb(msg):
    global robot, lwheel_speed_mm_sec, rwheel_speed_mm_sec

    lwheel_speed_mm_sec = msg.data
    robot.motors.set_wheel_motors(lwheel_speed_mm_sec, rwheel_speed_mm_sec)

def rwheel_desired_rate_cb(msg):
    global robot, lwheel_speed_mm_sec, rwheel_speed_mm_sec

    rwheel_speed_mm_sec = msg.data
    robot.motors.set_wheel_motors(lwheel_speed_mm_sec, rwheel_speed_mm_sec)

# subscribers
lwheel_desired_rate_subscriber = rospy.Subscriber("~lwheel_desired_rate", Int32, lwheel_desired_rate_cb)
rwheel_desired_rate_subscriber = rospy.Subscriber("~rwheel_desired_rate", Int32, rwheel_desired_rate_cb)

lwheel_ticks_total = 0
rwheel_ticks_total = 0

# init Vector services
vector_service = vector.VectorService(robot)
animation = anim.Animation(robot) # share robot object

while not rospy.is_shutdown():
    if robot.left_wheel_speed_mmps > 0:
        lwheel_ticks_total += robot.left_wheel_speed_mmps / RATE

    if robot.right_wheel_speed_mmps > 0:
        rwheel_ticks_total += robot.right_wheel_speed_mmps / RATE

    # publish number of estimated ticks for each wheel in ratio of 1000 ticks per meter
    lwheel_ticks_publisher.publish(data=lwheel_ticks_total)
    rwheel_ticks_publisher.publish(data=rwheel_ticks_total)

    rate.sleep()

robot.disconnect()