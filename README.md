# vector_ros
This repository contains an *unofficial* ROS package for [Anki Vector](https://www.anki.com/en-us/vector) that I started as a small side project after finishing several online ROS courses. This package is essentially a wrapping of core Vector functions from [Vector 
SDK](https://github.com/anki/vector-python-sdk) as ROS topics, services and actions(full list below). In order to showcase the package I wrote a simple [red ball tracking node](https://github.com/betab0t/vector_ros/blob/develop/nodes/simple_ball_tracker_node.py) which subscribes to the camera feed coming from Vector, locates the red ball using cv_bridge/OpenCV and publish Twist messages to move the robot accurdenly as you can see in the following video:

<p align="center">
  <a target="_blank" href="http://www.youtube.com/watch?v=XxaOyA-M3U4">
    <img src="http://img.youtube.com/vi/XxaOyA-M3U4/0.jpg">
  </a>
</p>

# Mentions
* ["RDP 039: Using Anki Vector robot with ROS with Omri Ben-Bassat"](http://www.theconstructsim.com/using-anki-vector-robot-ros-omri-ben-bassat) :robot: :tv:

# General Overview
Vector ROS project is actually divided into three separated packages, which are:
## vector_ros
* Main package, contains message service descriptors and example nodes, such as a red ball tracker.
## [vector_ros_driver](https://github.com/betab0t/vector_ros_driver)
* Physical / "real" robot driver node - this node does the actual interface to Vector using Vector Python SDK.
* Notice this package was developed using Python 3.6 to work with Vector's SDK. :snake:
* Offers easy deployment using Docker! :whale:
## [cozmo_simulation](https://bitbucket.org/theconstructcore/cozmo_simulation/)
* Simulated robot package for both Cozmo and Vector, created by the guys at [The Construct](http://theconstructsim.com) so we can use Vector in Gazebo.
* [Video tutorial showing spawn simulated Vector in ROS Developers Studio](http://www.theconstructsim.com/morpheus-chair-vector-ros-simulation-t2-ep-3/)

# Setup With Physical Robot
You can find the full setup instructions at [vector_ros_driver](https://github.com/betab0t/vector_ros_driver).

# Topics
* `/vector/camera`  *(sensor_msgs/Image)*

Vector camera feed.

* `/vector/cmd_vel` *(geometry_msgs/Twist)*

Move Vector around.

# Services

* `/vector/battery_state`

* `/vector/set_head_angle`

* `/vector/set_lift_height`

* `/vector/anim_list`

* `/vector/say_text`

# Actions

* `/vector/play_animation`

Play animation by name.

# Examples
## View single image from camera
```sh
beta_b0t@home:~$ rosrun image_view image_saver image:=/vector/camera
[ INFO] [1550425113.646567813]: Saved image left0000.jpg
[ INFO] [1550425113.752592532]: Saved image left0001.jpg
[ INFO] [1550425113.848999553]: Saved image left0002.jpg
...
(Ctrl+C)
...
beta_b0t@home:~$ eog left0000.jpg
```

## Set head angle
```sh
beta_b0t@home:~$ rosservice call /vector/set_head_angle "deg: 45.0"
```

## Say text
```sh
beta_b0t@home:~$ rosservice call /vector/say_text "text: 'hello world'"
```

## Play animation 
```sh
beta_b0t@home:~$ rostopic pub /vector/play_animation/goal vector_ros/PlayAnimationActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  anim: 'anim_turn_left_01'"
```

# FAQ
- **Why isn't this XX from Vector SDK supported?** Well, I didn't wrap all the functions from the SDK - only the main ones as i see it. Yet, if you found a missing function that you need/would like to see as part of vector_ros, please consider opening a [new issue](https://github.com/betab0t/vector_ros/issues/new) with your proposal.
