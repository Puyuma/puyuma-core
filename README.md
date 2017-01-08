#Xenobot

A lightweight replica of MIT duckietown project.

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

###4. Setup the rc.local file (Prevent motor rotate before activate the controller)

```
sudo cp rc.local /etc/rc.local
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

###3. Color detector calibration

```
roslaunch xenobot activate_controller.launch veh:=machine_name calibrate:=true
```

##Activating the controller

```
roslaunch xenobot activate_controller.launch veh:=machine_name
```

##Analysing

###Scatter plot

```
rosrun xenobot scatter_view_node.py
```

##GDB

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

##Trobleshooting

**Authentication to remote computer[ubuntu@192.168.0.102:22] failed.
A common cause of this error is a missing key in your authorized_keys file.**

Using ssh-copy-id could solve the problem:

```
ssh-copy-id xenbot_user@xenobot_address
```
