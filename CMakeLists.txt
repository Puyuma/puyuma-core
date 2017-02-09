cmake_minimum_required(VERSION 2.8.3)
project(xenobot)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(
    catkin REQUIRED COMPONENTS
    cv_bridge
    sensor_msgs
    std_msgs
    roscpp
	message_generation
	apriltags
)

find_package(Curses REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
    FILES
    threshold_setting.msg
    wheel_command.msg
    segmentArray.msg
    segment.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


########################################################################################
#Color threshold calibration

set(
    COLOR_THRESHOLD_INC
    ./calibration/color_threshold
)

set(
    COLOR_THRESHOLD_SRC
    ./calibration/color_threshold/main.cpp
)

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${COLOR_THRESHOLD_INC})
add_executable(color_threshold_calibration ${COLOR_THRESHOLD_SRC})
target_link_libraries (color_threshold_calibration ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp raspicam_cv raspicam)
add_dependencies(color_threshold_calibration ${PROJECT_NAME}_generate_messages_cpp)

########################################################################################
#Intrinsic calibration (Provide raw image)

set(
    INTRINSIC_CALIBRATION_INC
    ./calibration/intrinsic_calibration
)

set(
    INTRINSIC_CALIBRATION_SRC
    ./calibration/intrinsic_calibration/main.cpp
)

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${INTRINSIC_CALIBRATION_INC})
add_executable(intrinsic_calibration ${INTRINSIC_CALIBRATION_SRC})
target_link_libraries (intrinsic_calibration ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp raspicam_cv raspicam)

########################################################################################
#Extrinsic calibration

set(
    EXTRINSIC_CALIBRATION_INC
    ./calibration/extrinsic_calibration
)

set(
    EXTRINSIC_CALIBRATION_SRC
    ./calibration/extrinsic_calibration/main.cpp
)

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${EXTRINSIC_CALIBRATION_INC})
add_executable(extrinsic_calibration ${EXTRINSIC_CALIBRATION_SRC})
target_link_libraries (extrinsic_calibration ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp raspicam_cv raspicam)

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
add_dependencies(joystick ${PROJECT_NAME}_generate_messages_cpp)

########################################################################################
#Object tracker

set(
    XENOBOT_INC
    ./system
    ./system/motor_driver
    ./common
)

set(
    XENOBOT_SRC
    ./common/xeno_math.cpp
    ./system/main.cpp
    ./system/lane_detector.cpp
    ./system/controller.cpp
    ./system/motor.cpp
    ./system/camera.cpp
	./system/apriltags_detector.cpp
)

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${XENOBOT_INC})
add_executable(activate_controller ${XENOBOT_SRC})
target_link_libraries (activate_controller ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} -lwiringPi yaml-cpp raspicam_cv raspicam)
add_dependencies(activate_controller ${PROJECT_NAME}_generate_messages_cpp)
