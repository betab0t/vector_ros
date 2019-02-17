# vector_ros
This repository contains my unofficial ROS package for [Anki Vector](https://www.anki.com/en-us/vector) home robot.

# Setup
## Docker Image
```sh
git clone https://github.com/betab0t/vector_ros
cd vector_ros
cp docker-compose-TEMPLATE.yml docker-compose.yml
nano docker-compose.yml
```
edit the following lines and save:
```yaml
vector_ip: <VECTOR_IP>
vector_name: <VECTOR_NAME>
vector_serial: <VECTOR_SERIAL> 
```

```sh
sudo docker-compose build --build-arg anki_user_email=<ANKI_ACCOUNT_EMAIL> --build-arg anki_user_password=<ANKI_ACCOUNT_PASSWORD>
sudo docker-compose up
```

# Topics
* `/vector/camera`  *(sensor_msgs/Image)*

Vector camera feed.

* `/lwheel_ticks` *(std_msgs/Int32)*

Cumulative encoder ticks of the left wheel. used by [diff_drive](https://github.com/merose/diff_drive) package.

* `/rwheel_ticks`  *(std_msgs/Int32)*

Cumulative encoder ticks of the right wheel. used by [diff_drive](https://github.com/merose/diff_drive) package.

* `/lwheel_rate`  *(std_msgs/Int32)*

Left wheel rotation rate. used by [diff_drive](https://github.com/merose/diff_drive) package.

* `/rwheel_rate`  *(std_msgs/Int32)*

Right wheel rotation rate. used by [diff_drive](https://github.com/merose/diff_drive) package.

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
- **[How do i find Vector's IP address?](https://developer.anki.com/vector/docs/troubleshooting.html#can-t-find-vector-s-ip-address)**

- **[How do i find Vector's name?](https://developer.anki.com/vector/docs/troubleshooting.html#can-t-find-robot-name)**

- **[How do i find Vector's serial number?](https://developer.anki.com/vector/docs/troubleshooting.html#can-t-find-serial-number)**

- **Why isn't this XX from Vector SDK supported?** Well, I didn't wrapped all the functions from the SDK - only the main ones as i see it. Yet, if you found a missing function that you need/would like to see as part of vector_ros, please consider opening a [new issue](https://github.com/betab0t/vector_ros/issues/new) with your proposal.

