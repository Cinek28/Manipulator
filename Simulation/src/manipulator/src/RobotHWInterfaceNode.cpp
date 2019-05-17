#include "RobotHWInterface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RobotHWInterface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    RobotHWInterface Robot(nh);
    ros::waitForShutdown();
    return 0;
}