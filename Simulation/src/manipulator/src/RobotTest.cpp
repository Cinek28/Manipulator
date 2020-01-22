#include "RobotHWInterface.h"

int main(int argc, char** argv)
{
    printf("Test node starting...");
    ros::init(argc, argv, "RobotTest");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    RobotHWInterface Robot(nh);

    printf("Movement\n");

    KDL::JntArray point(6);
    point(0) = 0.1;
    point(1) = 0.1;
    point(2) = 0.1;
    point(3) = 0.0;
    point(4) = 0.0;
    point(5) = 0.0;

    printf("Movement\n");

    Robot.setCartPos(&point, 1, 6);

    printf("Test end\n");

    ros::waitForShutdown();
    return 0;
}