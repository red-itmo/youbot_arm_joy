cmake_minimum_required(VERSION 2.8.3)
project(youbot_arm_joy)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        sensor_msgs
        brics_actuator
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    joystick_control
    src/joystick_control.cpp
    src/JoystickControl.cpp
)

target_link_libraries(
    joystick_control
    ${catkin_LIBRARIES}
)