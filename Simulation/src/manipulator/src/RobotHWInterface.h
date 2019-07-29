#ifndef MANIPULATOR_HW
#define MANIPULATOR_HW

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

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

    bool isRobotConnected(){return false;};

protected:
    ros::NodeHandle nodeHandle;
    ros::Timer nonRealtimeTask;
    ros::Duration controlPeriod;
    ros::Duration elapsedTime;

    ros::Subscriber jointStateSub;
    ros::Subscriber twistSub;
    ros::Publisher poseStampedPub;
    ros::Publisher jointStatePub;
    ros::Publisher commandPub[6];

    PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
    std::shared_ptr<controller_manager::ControllerManager> controllerManager;

    void newVelCallback(const geometry_msgs::Twist &msg);
    geometry_msgs::PoseStamped solveDirectKinematics(const sensor_msgs::JointState &msg);
    KDL::JntArray solveIndirectKinematics(const geometry_msgs::Twist &msg);

    KDL::JntArray jointVelocities;
    KDL::Chain kinematicChain;

    double p_error_, v_error_, e_error_;
    double rate;
};

#endif //MANIPULATOR_HW