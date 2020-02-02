#include "RobotHWInterface.h"

int main(int argc, char** argv)
{
    bool isCubic = false; 

    if(argc > 1 && argv[1] == "cubic")
        isCubic = true;

    ROS_INFO("Test node starting...");

    ros::init(argc, argv, "RobotTest");
    ros::NodeHandle nh;

    std::vector<double> cartStartPosPoints;
    std::vector<double> cartPosPoints;
    
    //First position:
    cartStartPosPoints.push_back(0.4);
    cartStartPosPoints.push_back(0.5);
    cartStartPosPoints.push_back(0.2);
    cartStartPosPoints.push_back(0.0);
    cartStartPosPoints.push_back(0.0);
    cartStartPosPoints.push_back(0.0);

    RobotHWInterface::setCartPos(nh, cartStartPosPoints, 1, 2.0, isCubic);

    //Second position:
    cartPosPoints.push_back(0.4);
    cartPosPoints.push_back(-0.5);
    cartPosPoints.push_back(0.5);
    cartPosPoints.push_back(0.0);
    cartPosPoints.push_back(0.0);
    cartPosPoints.push_back(0.0);

    RobotHWInterface::setCartPos(nh, cartPosPoints, 1, 6.0, isCubic);

    ROS_INFO("Test end\n");

    return 0;
}