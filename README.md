#Xenobot

Inspired by MIT, this is a lightweight replica of MIT duckietown project.

Our goal is to build a low cost self-driving car based on realtime Linux (patched by Xenomai)

#Demo videos

[![lane_following](https://github.com/ncku-ros2-research/xenobot/blob/master/materials/demo_video1.jpeg?raw=true)](https://www.youtube.com/watch?v=84MXc0_F61o)

[![rviz](https://github.com/ncku-ros2-research/xenobot/blob/master/materials/demo_video2.jpeg?raw=true)](https://www.youtube.com/watch?v=XK602hzbORY&feature=youtu.be)

##Installation

Connecting to your raspberry pi using ssh then follow the instructions.

###1.Raspicam

```
git clone https://github.com/ncku-ros2-research/raspicam.git
cd raspicam/
mkdir build
cd build
cmake ..
sudo make install
```

###2.WiringPi

```
git clone https://github.com/ncku-ros2-research/WiringPi.git
cd WiringPi
./build
```

###3.Xenobot

```
cd $(YOUR_CATKIN_WS)/src
git clone https://github.com/ncku-ros2-research/xenobot.git
cd $(YOUR_CATKIN_WS)
catkin_make
```

##Calibration

You must do the calibration before first time initiating the controller.

###1. Intrinsic calibration

```
#Terminal A
rosrun xenobot intrinsic_calibration

#Terminal B
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.031 image:=/xenobot/raw_image --no-service-check
```

###2. Extrinsic calibration

```
roslaunch xenobot extrinsic_calibration.launch veh:=machine_name
```

###3. Color detector calibration

```
pi$ roslaunch xenobot activate_controller.launch veh:=machine_name cal:=true

pc$ rosrun xenobot color_threshold_calibration _color:=[yellow | white | both]
```
* inner == yellow
* outer == white

##Activating the controller

```
roslaunch xenobot activate_controller.launch veh:=machine_name
```

###4. Setup .bashrc

Add:

```
. /opt/ros/kinetic/setup.bash
. ~/catkin_ws/devel/setup.bash
export ROS_IP=`hostname -I`
alias play="roslaunch xenobot activate_controller.launch veh:=colin calibrate:=1"
alias stop=". ~/catkin_ws/src/xenobot/halt_motor.sh"
```

##Analysing

###Scatter plot

```
rosrun xenobot scatter_view_node.py
```

##Debug with GDB

1. Compile binaries with debug symbol

```
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

2. Activate controller with gdbserver

```
#Terminal A
roslaunch xenobot activate_controller_gdb.launch veh:=machine_name cal:=true
```

3. Launch gdb

```
#Terminal B
make gdbauto
```

##Trobleshooting

**Authentication to remote computer[ubuntu@192.168.0.102:22] failed.
A common cause of this error is a missing key in your authorized_keys file.**

Using ssh-copy-id could solve the problem:

```
ssh-copy-id xenbot_user@xenobot_address
```
