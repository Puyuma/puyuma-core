cmake_minimum_required(VERSION 2.8.3)
project(realtime_duckie)

find_package(
    catkin REQUIRED COMPONENTS
    cv_bridge
    sensor_msgs
    std_msgs
    roscpp
    genmsg
)
 
find_package(OpenCV REQUIRED)

generate_messages(DEPENDENCIES std_msgs)

########################################################################################
#Object tracker

set(
    OBJECT_TRACKING_INC
    ./camera
)

set(
    OBJECT_TRACKING_SRC
    ./camera/main.cpp
)

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${OBJECT_TRACKING_INC})
add_executable(camera_node ${OBJECT_TRACKING_SRC})
target_link_libraries (camera_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp raspicam_cv raspicam)
