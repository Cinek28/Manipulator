#ifndef MANIPULATORHW_BASE_H
#define MANIPULATORHW_BASE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include <controller_manager/controller_manager.h>

#include <ros/ros.h>
#include <kdl/kdl.hpp>

//Hardware interface for manipulator:
class RobotHWInterfaceBase : public hardware_interface::RobotHW
{
protected:
    // Interfaces
    hardware_interface::JointStateInterface jointStateInt;
    hardware_interface::PositionJointInterface positionJointInt;
    hardware_interface::VelocityJointInterface velocityJointInt;
    hardware_interface::EffortJointInterface effortJointInt;

    // Shared memory
    int numJoints;

    std::vector<std::string> jointNames;

    KDL::JntArray jointPosition;
    KDL::JntArray jointVelocity;
    KDL::JntArray jointEffort;

    KDL::JntArray jointPositionCommands;
    KDL::JntArray jointVelocityCommands;
    KDL::JntArray jointEffortCommands;
};

#endif //MANIPULATORHW_BASE_H