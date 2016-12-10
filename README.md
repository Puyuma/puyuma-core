#Xenobot

A lightweight replica of MIT duckietown project

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

##Trobleshooting

**Authentication to remote computer[ubuntu@192.168.0.102:22] failed.
A common cause of this error is a missing key in your authorized_keys file.**

Using ssh-copy-id could solve the problem:

```
ssh-copy-id xenbot_user@xenobot_address
```
