# Puyuma-core

Core components of self-driving car system, building a complete open source solution
for low-cost hardware along with Linux real-time extensions.

## Demo videos

[![lane_following](https://github.com/ncku-ros2-research/xenobot/blob/master/materials/demo_video1.jpeg?raw=true)](https://www.youtube.com/watch?v=84MXc0_F61o)

[![rviz](https://github.com/ncku-ros2-research/xenobot/blob/master/materials/demo_video2.jpeg?raw=true)](https://www.youtube.com/watch?v=XK602hzbORY&feature=youtu.be)

Licensing
---------
`puyuma-core` is freely redistributable under the two-clause BSD License.
Use of this source code is governed by a BSD-style license that can be found
in the `LICENSE` file.

## Installation

Connecting to your raspberry pi using ssh then follow the instructions.

### 0.Install dependency

Install ROS kinetic ,or indigo from
http://wiki.ros.org/kinetic/Installation/Ubuntu

```
sudo apt install libncurses5-dev libopencv-dev ros-ROS_DISTRO-cv-bridge
```

### 1.Raspicam

```
git clone https://github.com/Puyuma/raspicam.git
cd raspicam/
mkdir build
cd build
cmake ..
sudo make install
```

### 2.WiringPi

```
git clone https://github.com/Puyuma/WiringPi.git
cd WiringPi
./build
```

### 3.Apriltags_cpp

```
cd $(YOUR_CATKIN_WS)/src
git clone https://github.com/Puyuma/apriltags.git
cd $(YOUR_CATKIN_WS)
catkin_make
```

### 4.Puyuma

```
cd $(YOUR_CATKIN_WS)/src
git clone https://github.com/Puyuma/puyuma-core.git
cd $(YOUR_CATKIN_WS)
catkin_make
```

## Calibration

You must do the calibration before first time initiating the controller.

### 1. Intrinsic calibration

```
#Terminal A
rosrun xenobot intrinsic_calibration

#Terminal B
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.031 image:=/xenobot/raw_image --no-service-check
```

### 2. Extrinsic calibration

```
roslaunch xenobot extrinsic_calibration.launch veh:=machine_name
```

### 3. Color detector calibration

```
pi$ roslaunch xenobot activate_controller.launch veh:=machine_name cal:=true

pc$ rosrun xenobot color_threshold_calibration _color:=[ 1 | 2 | 3 ]

//[1: yellow | 2: white | 3: both]
```
* inner == yellow
* outer == white

### 4. Setup .bashrc

Add:

```
. /opt/ros/kinetic/setup.bash
. $(YOUR_CATKIN_WS)/devel/setup.bash
export ROS_IP=`hostname -I`
alias play="roslaunch xenobot activate_controller.launch veh:=VEHICLE_NAME calibrate:=true"
alias stop=". ~/catkin_ws/src/xenobot/halt_motor.sh"
```

## Activating the controller

```
roslaunch xenobot activate_controller.launch veh:=VEHICLE_NAME calibrate:=true
# or simply
play
```


## Analysing

### Scatter plot

```
rosrun xenobot scatter_view_node.py
```

### GDB

1. Use **ps** command find the process ID

```
ps aux
```

2. Enter GDB or CGDB

```
cgdb
```

3. Attatch the process

```
attach **PROCESS_ID**
```

## Trobleshooting

**Authentication to remote computer[ubuntu@192.168.0.102:22] failed.
A common cause of this error is a missing key in your authorized_keys file.**

Using ssh-copy-id could solve the problem:

```
ssh-copy-id xenbot_user@xenobot_address
```
