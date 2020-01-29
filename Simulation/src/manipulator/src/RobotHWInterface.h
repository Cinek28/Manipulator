#ifndef MANIPULATOR_HW
#define MANIPULATOR_HW

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include "RobotHWInterfaceBase.h"
#include "RobotSerialComm.h"

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;


enum CONTROLLER_TYPE
{
    VELOCITY_CONTROLLER = 0,
    POSITION_CONTROLLER
};

enum INTERPOLATION_TYPE
{
    LINEAR = 0,
    CUBIC
};

class RobotHWInterface : public RobotHWInterfaceBase
{
public:
    RobotHWInterface(ros::NodeHandle &nh);
    ~RobotHWInterface();

    void init();

    bool isRobotConnected(){return robot.isOpened();};

    bool setCartPos(const KDL::JntArray *pos,
        size_t pointsNr,
        double timeSec,
        INTERPOLATION_TYPE interp = LINEAR,
        unsigned interpPoints = 10);

    bool setJntPos(const KDL::JntArray &pos,
        size_t pointsNr,
        double timeSec,
        INTERPOLATION_TYPE interp = LINEAR,
        unsigned interpPoints = 10);

private:
    ros::NodeHandle nodeHandle;
    ros::Timer nonRealtimeTask;
    ros::Duration controlPeriod;
    ros::Duration elapsedTime;

    ManipulatorCmd mode = JOINT;
    CONTROLLER_TYPE controller = POSITION_CONTROLLER;
    bool isMoving = false;

    KDL::JntArray jointVelocities;
    KDL::JntArray jointPositions;

    ros::Subscriber jointStateSub;
    ros::Subscriber twistSub;
    ros::Subscriber posSub;
    ros::Subscriber trajSub;
    ros::Publisher poseStampedPub;
//    ros::Publisher jointStatePub;
    ros::Publisher commandPub[6];
    ros::Publisher commandPubVelPos[6];
    ros::Publisher commandTrajectoryPub;

    bool setVel(const KDL::JntArray &vel);
    bool setPos(const KDL::JntArray &pos);

    void read();
    void update(const ros::TimerEvent &e);
    void write(ros::Duration elapsed_time);

    void newRobotState(const sensor_msgs::JointState &msg);

protected:

    PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
    std::shared_ptr<controller_manager::ControllerManager> controllerManager;

    KDL::Chain kinematicChain;

    RobotSerialComm robot;

    double p_error_, v_error_, e_error_;
    double rate;

    void newVelCallback(const geometry_msgs::Twist &msg);
    void newPosCallback(const geometry_msgs::Twist &msg);
    void newTrajCallback(const geometry_msgs::Twist &msg);

    KDL::Frame solveDirectKinematics(const KDL::JntArray &pos);
    KDL::JntArray solveIndirectPosKinematics(const KDL::JntArray &vel);
    KDL::JntArray solveIndirectVelKinematics(const KDL::JntArray &pos);
};

#endif //MANIPULATOR_HW