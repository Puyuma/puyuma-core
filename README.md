#Xenobot

A lightweight replica of MIT duckietown project

##Calibration

1. Intrinsic calibration

2. Extrinsic calibration

3. Color detector calibration

##Intrinsic calibration

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.031 image:=/realtime_duckie/raw_image --no-service-check

##Color thresholding calibration

roslaunch xenobot activate_controller.launch veh:=machine_name calibrate:=true

##Activate controller

roslaunch xenobot activate_controller.launch veh:=machine_name
