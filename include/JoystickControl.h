#ifndef _JOYSTICK_CONTROL_
#define _JOYSTICK_CONTROL_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>
#include <cstdio>
#include <vector>

class JoystickControl{

    ros::NodeHandle nh, nh_for_params;
    ros::Subscriber joystick_reader;
    ros::Subscriber arm_state_reader;
    ros::Publisher arm_commander;
    ros::Publisher gripper_commander;

    bool gripper_opened;
    double arm_state[5];
    int arm_direction[5];
    double safe_angle;
    double arm_state_min[5];
    double arm_state_max[5];

    void joystickCallback(const sensor_msgs::Joy &msg);
    void jointStateCallback(const sensor_msgs::JointState &msg);
public:

    JoystickControl();
    ~JoystickControl();
    void doYourWork();
};

#endif //_JOYSTICK_CONTROL_
