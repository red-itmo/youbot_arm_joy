#include <JoystickControl.h>

int main (int argc, char **argv){
    ros::init(argc, argv, "joystick_control");
    JoystickControl node;
    node.doYourWork();
    return 0;
}
