#include "RobotHWInterface.h"

int main(int argc, char** argv)
{
    ROS_INFO("Control node starting...");
    ros::init(argc, argv, "RobotHWInterface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    RobotHWInterface Robot(nh);
    ros::waitForShutdown();
    return 0;
}