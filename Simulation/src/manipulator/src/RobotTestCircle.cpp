#include "RobotHWInterface.h"

int main(int argc, char** argv)
{
    bool isCubic = false;
    int points = 10;

    if(argc > 1 && argv[1] == "cubic")
        isCubic = true;
    if(argc == 3)
        points = atoi(argv[2]);

    ROS_INFO("Test node starting...");

    ros::init(argc, argv, "RobotTestCircle");
    ros::NodeHandle nh;

    std::vector<double> cartPosPoints;
    std::vector<double> cartCirclePosPoints;

    double r = 0.15;
    double d = 0.6;
    double alpha = 2 * M_PI / points;

    cartPosPoints.push_back(d + 0.15);
    cartPosPoints.push_back(0.0);
    cartPosPoints.push_back(0.4);
    cartPosPoints.push_back(0.0);
    cartPosPoints.push_back(0.0);
    cartPosPoints.push_back(0.0);

    RobotHWInterface::setCartPos(nh, cartPosPoints, 1, 4.0, isCubic);

    for(int i = 0; i < points; ++i)
    {
        cartCirclePosPoints.push_back(d + r * cos((i + 1) * alpha));
        cartCirclePosPoints.push_back(r * sin((i + 1) * alpha));
        cartCirclePosPoints.push_back(0.4);
        cartCirclePosPoints.push_back(0.0);
        cartCirclePosPoints.push_back(0.0);
        cartCirclePosPoints.push_back(0.0);
    }

    RobotHWInterface::setCartPos(nh, cartCirclePosPoints, points, 12.0, isCubic);

    ROS_INFO("Test end\n");

    return 0;
}