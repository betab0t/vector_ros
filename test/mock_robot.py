#!/usr/bin/env python3.6

import rospy
import concurrent

from PIL import Image

class MockMotorComponent(object):
    def __init__(self):
        self._left_wheel_speed = 0
        self._right_wheel_speed = 0

    def set_wheel_motors(self, left_wheel_speed, right_wheel_speed):
        self._left_wheel_speed = left_wheel_speed
        self._right_wheel_speed = right_wheel_speed

    def get_wheel_motors(self):
        return self._left_wheel_speed, self._right_wheel_speed

class MockAnim(object):
    def __init__(self):
        self.anim_list = ['anim_turn_left_01']

    def play_animation(self, animation):
        res = concurrent.futures.Future()
        res.set_result(rospy.sleep(1))
        return res

class MockCamera(object):
    def __init__(self):
        self.latest_image = Image.new('RGB', (1280, 720))

class MockBehavior(object):
    def set_head_angle(self, deg):
        return

    def set_lift_height(self, height):
        return

class MockRobot(object):

    class BatteryState(object):
        def __init__(self, battery_volts=3.0, battery_level=1, is_charging=False, is_on_charger_platform=False, suggested_charger_sec=100.0):
            self.battery_volts = battery_volts
            self.battery_level = battery_level
            self.is_charging = is_charging
            self.is_on_charger_platform = is_on_charger_platform
            self.suggested_charger_sec = suggested_charger_sec

    def __init__(self, **kargs):
        self.motors = MockMotorComponent()
        self.camera = MockCamera()
        self.anim = MockAnim()
        self.behavior = MockBehavior()

    def connect(self):
        rospy.loginfo("mock robot connected!")

    @property
    def left_wheel_speed_mmps(self):
        return self.motors.get_wheel_motors()[0]

    @property
    def right_wheel_speed_mmps(self):
        return self.motors.get_wheel_motors()[1]

    def get_battery_state(self):
        res = concurrent.futures.Future()
        res.set_result(MockRobot.BatteryState())
        return res

    def say_text(self, text):
        return