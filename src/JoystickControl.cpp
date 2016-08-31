#include <JoystickControl.h>


JoystickControl::JoystickControl(): nh_for_params("~"){
    safe_angle = 0.3;
    arm_state_min[0] =  0.00;
    arm_state_min[1] =  0.00;
    arm_state_min[2] = -5.18;
    arm_state_min[3] =  0.00;
    arm_state_min[4] =  0.00;
    arm_state_max[0] =  5.90;
    arm_state_max[1] =  2.70;
    arm_state_max[2] =  0.00;
    arm_state_max[3] =  3.58;
    arm_state_max[4] =  5.85;

    gripper_opened = false;
    joystick_reader = nh.subscribe("joy", 10, &JoystickControl::joystickCallback, this);
    arm_commander = nh.advertise<brics_actuator::JointVelocities>("arm_1/arm_controller/velocity_command", 10);
    gripper_commander = nh.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 10);
    arm_state_reader = nh.subscribe("joint_states", 10, &JoystickControl::jointStateCallback, this);
}



JoystickControl::~JoystickControl(){
    joystick_reader.shutdown();
    arm_commander.shutdown();
    gripper_commander.shutdown();
}

void JoystickControl::jointStateCallback(const sensor_msgs::JointState& msg){
    brics_actuator::JointVelocities arm_velocities;
    brics_actuator::JointValue value;
    value.unit = "s^-1 rad";
    value.value = 0.0;

    int i;
    for(i = 0; i < 5; i++){
        arm_state[i] = msg.position[i];
        char digit[2];
        std::sprintf(digit, "%d", i+1);
        value.joint_uri = "arm_joint_" + std::string(digit);
        if(((arm_state_max[i] -  arm_state[i] < safe_angle) && (arm_direction[i] == 1)) ||
            (( arm_state[i] - arm_state_min[i] < safe_angle) && (arm_direction[i] == -1)))
                arm_velocities.velocities.push_back(value);
    }

    arm_commander.publish(arm_velocities);
}



void JoystickControl::joystickCallback(const sensor_msgs::Joy &msg){

    brics_actuator::JointPositions gripper_state;
    brics_actuator::JointValue value;
    value.unit = "m";
    if((msg.buttons[1] == 1) && (msg.buttons[3] == 0) && (!gripper_opened)){
        gripper_opened = true;
        value.joint_uri = "gripper_finger_joint_l";
        value.value = 0.01;
        gripper_state.positions.push_back(value);
        value.joint_uri = "gripper_finger_joint_r";
        gripper_state.positions.push_back(value);
        gripper_commander.publish(gripper_state);
    }
    else if((msg.buttons[1] == 0) && (msg.buttons[3] == 1) && gripper_opened){
        gripper_opened = false;
        value.joint_uri = "gripper_finger_joint_l";
        value.value = 0.00;
        gripper_state.positions.push_back(value);
        value.joint_uri = "gripper_finger_joint_r";
        gripper_state.positions.push_back(value);
        gripper_commander.publish(gripper_state);
    }


    brics_actuator::JointVelocities arm_velocities;
    value.unit = "s^-1 rad";


    value.joint_uri = "arm_joint_5";
    if((msg.buttons[0] == 1) && (msg.buttons[2] == 0) && (arm_state_max[4] - arm_state[4] > safe_angle) ){
        value.value = 0.3;
        arm_direction[4] = 1;
    }
    else if((msg.buttons[0] == 0) && (msg.buttons[2] == 1) && (arm_state[4] - arm_state_min[4] > safe_angle)){
        value.value = -0.3;
        arm_direction[4] = -1;
    }
    else{
        value.value = 0.0;
        arm_direction[4] = 0;
    }
    arm_velocities.velocities.push_back(value);


    value.joint_uri = "arm_joint_4";
    if((msg.buttons[5] == 1) && (msg.buttons[7] == 0) && (arm_state_max[3] - arm_state[3] > safe_angle)){
        value.value = 0.3;
        arm_direction[3] = 1;
    }
    else if((msg.buttons[5] == 0) && (msg.buttons[7] == 1) && (arm_state[3] - arm_state_min[3] > safe_angle)){
        value.value = -0.3;
        arm_direction[3] = -1;
    }
    else{
        value.value = 0.0;
        arm_direction[3] = 0;
    }
    arm_velocities.velocities.push_back(value);


    value.joint_uri = "arm_joint_3";
    if((msg.buttons[4] == 1) && (msg.buttons[6] == 0) && (arm_state_max[2] - arm_state[2] > safe_angle)){
        value.value = 0.3;
        arm_direction[2] = 1;
    }
    else if((msg.buttons[4] == 0) && (msg.buttons[6] == 1) && (arm_state[2] - arm_state_min[2] > safe_angle)){
        value.value = -0.3;
        arm_direction[2] = -1;
    }
    else{
        value.value = 0.0;
        arm_direction[2] = 0;
    }
    arm_velocities.velocities.push_back(value);


    value.joint_uri = "arm_joint_2";
    if(msg.axes[5] == 1.0 && (arm_state_max[1] - arm_state[1] > safe_angle)){
        value.value = 0.3;
        arm_direction[1] = 1;
    }
    else if(msg.axes[5] == -1.0 && (arm_state[1] - arm_state_min[1] > safe_angle)){
        value.value = -0.3;
        arm_direction[1] = -1;
    }
    else{
        value.value = 0.0;
        arm_direction[1] = 0;
    }
    arm_velocities.velocities.push_back(value);


    value.joint_uri = "arm_joint_1";
    if(msg.axes[4] == 1.0 && (arm_state_max[0] - arm_state[0] > safe_angle)){
        value.value = 0.3;
        arm_direction[0] = 1;
    }
    else if(msg.axes[4] == -1.0 && (arm_state[0] - arm_state_min[0] > safe_angle)){
        value.value = -0.3;
        arm_direction[0] = -1;
    }
    else{
        value.value = 0.0;
        arm_direction[0] = 0;
    }
    arm_velocities.velocities.push_back(value);

    arm_commander.publish(arm_velocities);
}



void JoystickControl::doYourWork(){
    ros::spin();
}
