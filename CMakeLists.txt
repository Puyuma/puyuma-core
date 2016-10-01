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

find_package(Curses REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(WiringPi REQUIRED)

#include_directories(${WIRINGPI_INCLUDE_DIRS})

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

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${CALIBRATION_INC})
add_executable(calibration_panel ${CALIBRATION_SRC})
target_link_libraries (calibration_panel ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp raspicam_cv raspicam)

########################################################################################
#JOYSTICK

set(
    JOYSTICK_INC
    ./joystick
)

set(
    JOYSTICK_SRC
    ./joystick/main.cpp
)

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${CURSES_INCLUDE_DIR} ${JOYSTICk_INC})
add_executable(joystick ${JOYSTICK_SRC})
target_link_libraries (joystick ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${CURSES_LIBRARIES} yaml-cpp raspicam_cv)

########################################################################################
#Object tracker

set(
    XENOBOT_INC
    ./system
    ./system/motor_driver
)

set(
    XENOBOT_SRC
    ./system/main.cpp
    ./system/lane_detector.cpp
    ./system/controller.cpp
)

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${XENOBOT_INC})
add_executable(system_node ${XENOBOT_SRC})
target_link_libraries (system_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} -lwiringPi yaml-cpp raspicam_cv raspicam)
