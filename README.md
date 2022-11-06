[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
# Overview
    This repo contains the programming assignment (ROS Publisher/Subscriber) for ENPM808x, ROS2.
* Name : Yashveer Jain
* UID : 119252864

## Assumption
Ros2 is already setup locally and have a workspace `ros2_ws`.

## build
* First Time build
```
cd ~/ros2_ws/src
git clone <repo>
cd ..
colcon build
. install/setup.bash
```
* After editing `beginner_tutorials` build:
```
colcon build --packages-select beginner_tutorials
. install/setup.bash
```

## Run
* publisher in one terminal (with loglevel start from debug)
```
ros2 run beginner_tutorials talker --ros-args --log-level debug
```
* subscriber in another terminal (with loglevel start from debug)
```
ros2 run beginner_tutorials listener  --ros-args --log-level debug
```

## Dependencies:
* rclcpp
* std_msgs
