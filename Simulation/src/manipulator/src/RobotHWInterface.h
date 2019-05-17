#ifndef MANIPULATOR_HW
#define MANIPULATOR_HW

#include "RobotHWInterfaceBase.h"

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

static const double POSITION_STEP_FACTOR = 10;
static const double VELOCITY_STEP_FACTOR = 10;

class RobotHWInterface : public RobotHWInterfaceBase
{
public:
    RobotHWInterface(ros::NodeHandle &nh);
    ~RobotHWInterface();
    void init();
    void update(const ros::TimerEvent &e);
    void read();
    void write(ros::Duration elapsed_time);

protected:
    ros::NodeHandle nodeHandle;
    ros::Timer nonRealtimeTask;
    ros::Duration controlPeriod;
    ros::Duration elapsedTime;
    PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
    double rate;
    std::shared_ptr<controller_manager::ControllerManager> controllerManager;
    double p_error_, v_error_, e_error_;
};

#endif //MANIPULATOR_HW