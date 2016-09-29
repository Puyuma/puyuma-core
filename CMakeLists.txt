cmake_minimum_required(VERSION 2.8.3)
project(xenobot)

find_package(
    catkin REQUIRED COMPONENTS
    cv_bridge
    sensor_msgs
    std_msgs
    roscpp
    genmsg
)
 
find_package(OpenCV REQUIRED)

add_message_files(
    FILES
    threshold_setting.msg
)

generate_messages(DEPENDENCIES std_msgs)

########################################################################################
#Calibration panel

set(
    CALIBRATION_INC
    ./calibration
)

set(
    CALIBRATION_SRC
    ./calibration/main.cpp
)

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${CALIBRATION__INC})
add_executable(calibration_panel ${CALIBRATION_SRC})
target_link_libraries (calibration_panel ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp raspicam_cv raspicam)

########################################################################################
#Object tracker

set(
    XENOBOT_INC
    ./camera
    ./camera/motor_driver
)

set(
    XENOBOT_SRC
    ./camera/main.cpp
    ./camera/lane_detector.cpp
    ./camera/controller.cpp
    ./camera/motor_driver/gnublin.cpp
)

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${XENOBOT_INC})
add_executable(camera_node ${XENOBOT_SRC})
target_link_libraries (camera_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp raspicam_cv raspicam)
