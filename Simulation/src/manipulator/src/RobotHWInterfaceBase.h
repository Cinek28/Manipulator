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

    // joint_limits_interface::EffortJointSaturationInterface effort_joint_saturation_interface_;
    // joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_limits_interface_;
    // joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
    // joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;
    // joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
    // joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limits_interface_;


    // Shared memory
    int numJoints;

    std::vector<std::string> jointNames;

    KDL::JntArray jointPosition;
    KDL::JntArray jointVelocity;
    KDL::JntArray jointEffort;

    KDL::JntArray jointPositionCommands;
    KDL::JntArray jointVelocityCommands;
    KDL::JntArray jointEffortCommands;
    // std::vector<double> jointLowerLimits;
    // std::vector<double> jointUpperLimits;
    // std::vector<double> jointEffortLimits;

};

#endif //MANIPULATORHW_BASE_H