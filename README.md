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
* publisher in one terminal
```
ros2 run beginner_tutorials talker
```
* subscriber in another terminal
```
ros2 run beginner_tutorials listener
```

## Dependencies:
* rclcpp
* std_msgs
