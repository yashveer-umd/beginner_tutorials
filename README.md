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
* Running service client (change `new_string_to_publish` to the desired string want to publish)
```
ros2 run beginner_tutorials server_client <new_string_to_publish> --ros-args --log-level debug
```
## Run with launch file
```
ros2 launch launch/custom_launch.py  enable_recording:=False frequency:=1
```
* Note: frequency must be integer value (hz).
* It will launch publisher and subscriber nodes, and server node within publisher.
* so, server_client node need to be called separately and message can be edited.
* `enable_recording` argument can either be `False` or `True`:
    - which enable ros2 bag for recording the topic output

## Working ros2 bag output:
### replay the recorded data from ros2 bag recording:
* In one terminal run:
    > ros2 bag play results/bag_output`
* In second terminal just spawn listener to get the output of the bag
    > ros2 run beginner_tutorials listener

* get the ros2 bag output info:
    > ros2 bag info results/bag_output
    - output:
    ```
    Files:             bag_output_0.db3
    Bag size:          17.0 KiB
    Storage id:        sqlite3
    Duration:          7.500s
    Start:             Nov 30 2022 00:32:57.319 (1669786377.319)
    End:               Nov 30 2022 00:33:04.819 (1669786384.819)
    Messages:          31
    Topic information: Topic: /chatter | Type: std_msgs/msg/String | Count: 31 | Serialization Format: c
    ```
## Working if tf2 
* run the talker with the tf2
    > ros2 run beginner_tutorials talker talk 0 0 1 0 0 0
    - This is defining the transformation of `talk` frame with respect to `world` frame, where `talk` frame is of distance `1` unit in z direction and no rotation with respect to `world` frame.
    - Note: here `world` is parent frame while `talk` is a child frame.
* In another terminal run
    > ros2 run tf2_ros tf2_echo world talk
    - output:
    ```
    At time 0.0
    - Translation: [0.000, 0.000, 1.000]
    - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
    - Rotation: in RPY (radian) [0.000, -0.000, 0.000]
    - Rotation: in RPY (degree) [0.000, -0.000, 0.000]
    - Matrix:
    1.000  0.000  0.000  0.000
    0.000  1.000  0.000  0.000
    0.000  0.000  1.000  1.000
    0.000  0.000  0.000  1.000

    ```

## Checks
### cpplint:
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```
### cppcheck:
```
cppcheck --enable=all --std=c++11 --check-config --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```

## Results:
* `bag_output` dir:
    - Contains the recording of the chatter data in sqlite format
* `frames_2022-11-29_12.11.33.pdf` output of the Tf2 (transform) after running command:
    > ros2 run tf2_tools view_frames
* Cpplint output
* Cppcheck output


## Dependencies:
* rclcpp
* std_msgs
* geometry_msgs
* tf2
* tf2_ros
