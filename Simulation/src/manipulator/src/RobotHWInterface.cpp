#include <sstream>
#include "RobotHWInterface.h"
// #include <ROBOTcpp/ROBOT.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

RobotHWInterface::RobotHWInterface(ros::NodeHandle &nh) : nodeHandle(nh)
{
    init();
    controllerManager.reset(new controller_manager::ControllerManager(this, nodeHandle));
    nodeHandle.param("/manipulator/hardware_interface/rate", rate, 0.1);
    // nonRealtimeTask = nodeHandle.createTimer(ros::Duration(1.0 / rate), nullptr, this);
}

RobotHWInterface::~RobotHWInterface()
{
}

void RobotHWInterface::init()
{
    // Get joint names
    nodeHandle.getParam("/ROBOT/hardware_interface/joints", jointNames);
    numJoints = jointNames.size();

    // Resize vectors
    jointPosition.resize(numJoints);
    jointVelocity.resize(numJoints);
    jointEffort.resize(numJoints);
    jointPositionCommands.resize(numJoints);
    jointVelocityCommands.resize(numJoints);
    jointEffortCommands.resize(numJoints);

    // Initialize Controller
    for (int i = 0; i < numJoints; ++i)
    {
        // ROBOTcpp::Joint joint = ROBOT.getJoint(jointNames[i]);

        // Create joint state interface
        JointStateHandle jointStateHandle(jointNames[i], &jointPosition[i], &jointVelocity[i], &jointEffort[i]);
        jointStateInt.registerHandle(jointStateHandle);

        // Create position joint interface
        JointHandle jointPositionHandle(jointStateHandle, &jointPositionCommands[i]);
        JointLimits limits;
        SoftJointLimits softLimits;
        getJointLimits(jointNames[i], nodeHandle, limits);
        PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
        positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
        positionJointInt.registerHandle(jointPositionHandle);

        // Create effort joint interface
        JointHandle jointEffortHandle(jointStateHandle, &jointEffortCommands[i]);
        effortJointInt.registerHandle(jointEffortHandle);
    }

    registerInterface(&jointStateInt);
    registerInterface(&positionJointInt);
    registerInterface(&effortJointInt);
    registerInterface(&positionJointSoftLimitsInterface);
}

void RobotHWInterface::update(const ros::TimerEvent &e)
{
    elapsedTime = ros::Duration(e.current_real - e.last_real);
    read();
    controllerManager->update(ros::Time::now(), elapsedTime);
    write(elapsedTime);
}

void RobotHWInterface::read()
{
    for (int i = 0; i < numJoints; i++)
    {
        // TODO: Read positions from robot
        jointPosition[i] = jointPosition[i];//ROBOT.getJoint(joint_names_[i]).read();
    }
}

void RobotHWInterface::write(ros::Duration elapsed_time)
{
    positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
    for (int i = 0; i < numJoints; i++)
    {
        // TODO: Write joints velocities/efforts
        //ROBOT.getJoint(joint_names_[i]).actuate(jointEffortCommands[i]);
    }
}