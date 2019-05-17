#include <RobotHW.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RobotHW");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    RobotHW Robot(nh);
    ros::spin();
    return 0;
}