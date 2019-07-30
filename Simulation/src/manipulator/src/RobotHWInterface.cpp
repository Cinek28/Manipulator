#include <sstream>
#include <std_msgs/Float64.h>
#include "RobotHWInterface.h"

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

    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Vector(0, 0, 0.13))));

    KDL::Frame frame = KDL::Frame(KDL::Rotation::RPY(-KDL::PI/2,0.0,0.0),KDL::Vector(0, 0.00175, 0.0705));
    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    frame = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,1.283),KDL::Vector(0, -0.6, 0.0));
    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    frame = KDL::Frame(KDL::Rotation::EulerZYX(0.0,-KDL::PI/2,0.0),KDL::Vector(0.408, 0.005, 0.0));
    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    frame = KDL::Frame(KDL::Rotation::EulerZYX(-KDL::PI/2,0.0,3*KDL::PI/2),KDL::Vector(0.0, 0.0, -0.129));
    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    frame = KDL::Frame(KDL::Rotation::EulerZYX(0.0,-KDL::PI/2,0.0),KDL::Vector(0.0643, 0.0, 0.0));
    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    frame = KDL::Frame(KDL::Rotation::EulerZYX(0.0,KDL::PI/2,0.0),KDL::Vector(0.0, 0.0, -0.15));
    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    jointVelocities = KDL::JntArray(kinematicChain.getNrOfJoints());

    // Sub geometry_msgs::Twist
    twistSub = nodeHandle.subscribe("/spacenav/twist",1,&RobotHWInterface::newVelCallback, this);

//    // Pub geometry_msgs/PoseStamped
//    poseStampedPub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);
//
//    jointStatePub = nodeHandle.advertise<sensor_msgs::JointState>("/joint_states", 1);

    // Pub external controller steering commands:
    commandPub[0] = nodeHandle.advertise<std_msgs::Float64>("/manipulator/shoulder_rotation_controller/command", 1);
    commandPub[1] = nodeHandle.advertise<std_msgs::Float64>("/manipulator/forearm_rotation_controller/command", 1);
    commandPub[2] = nodeHandle.advertise<std_msgs::Float64>("/manipulator/arm_rotation_controller/command", 1);
    commandPub[3] = nodeHandle.advertise<std_msgs::Float64>("/manipulator/wrist_rotation_controller/command", 1);
    commandPub[4] = nodeHandle.advertise<std_msgs::Float64>("/manipulator/wrist_pitch_controller/command", 1);
    commandPub[5] = nodeHandle.advertise<std_msgs::Float64>("/manipulator/gripper_rotation_controller/command", 1);

    std_msgs::Float64 initial_vel;
    initial_vel.data = 0.0;

    for(int i = 0; i < 6; ++i)
        commandPub[i].publish(initial_vel);

    nonRealtimeTask = nodeHandle.createTimer(ros::Duration(1.0 / rate), &RobotHWInterface::update, this);
}

RobotHWInterface::~RobotHWInterface()
{
}

void RobotHWInterface::init()
{
    // Get joint names
    nodeHandle.getParam("/manipulator/hardware_interface/joints", jointNames);
    numJoints = jointNames.size();
    ROS_INFO("Number of joints: %d", numJoints);

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
        // Create joint state interface
        JointStateHandle jointStateHandle(jointNames[i], &jointPosition[i], &jointVelocity[i], &jointEffort[i]);
        jointStateInt.registerHandle(jointStateHandle);

        // Create position joint interface
        JointHandle jointPositionHandle(jointStateHandle, &jointPositionCommands[i]);
//        JointLimits limits;
//        SoftJointLimits softLimits;
//        getJointLimits(jointNames[i], nodeHandle, limits);
//        PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
//        positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
        positionJointInt.registerHandle(jointPositionHandle);

        //Create velocity joint interface:
        JointHandle jointVelocityHandle(jointStateHandle, &jointVelocityCommands[i]);
        velocityJointInt.registerHandle(jointVelocityHandle);

        // Create effort joint interface
        JointHandle jointEffortHandle(jointStateHandle, &jointEffortCommands[i]);
        effortJointInt.registerHandle(jointEffortHandle);
    }

    registerInterface(&jointStateInt);
    registerInterface(&velocityJointInt);
    registerInterface(&positionJointInt);
    registerInterface(&effortJointInt);
//    registerInterface(&positionJointSoftLimitsInterface);

    robot.openPort(0, BAUDRATE);
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
    ManipulatorMsg msg;
    
    if(isRobotConnected() && robot.readData(&msg))
    {
        for (int i = 0; i < numJoints; i++)
        {
            jointVelocity[i] = (double)(msg.params[i] | (msg.params[i+1] << 8));
        }
    }
}

void RobotHWInterface::write(ros::Duration elapsed_time)
{
//    positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
    if(!isRobotConnected() || mode == IDLE)
    {
        for(int i = 0; i < numJoints; ++i)
        {
            ROS_INFO("Setting command to %s, %d: %f",jointNames[i].c_str(), i, jointVelocities(i));
            velocityJointInt.getHandle(jointNames[i]).setCommand(jointVelocities(i));
        }
    }
    else
    {   ManipulatorMsg msg;
        uint16_t velocity = 0;
        msg.type = mode;
        msg.length = 2*numJoints;
        msg.checksum = msg.type + msg.length;
        for (int i = 0; i < numJoints; i++)
        {
            // TODO: Write joints velocities/efforts
            msg.params[i] = (uint8_t)jointVelocities(i);
            msg.params[i+1] = (uint8_t)(jointVelocities(i) >> 8);
            msg.checksum += msg.params[i] + msg.params[i+1];
        }
        msg.checksum ~= msg.checksum;
        robot.sendData(&msg);
    }
}

KDL::JntArray RobotHWInterface::solveIndirectKinematics(const geometry_msgs::Twist &msg) {

    KDL::Twist temp_twist;
    temp_twist.vel.x(msg.linear.x);
    temp_twist.vel.y(msg.linear.y);
    temp_twist.vel.z(msg.linear.z);
    temp_twist.rot.x(msg.angular.x);
    temp_twist.rot.y(msg.angular.y);
    temp_twist.rot.z(msg.angular.z);

    //Create joint array
    KDL::JntArray joint_positions = KDL::JntArray(kinematicChain.getNrOfJoints());
    for(int i = 0; i < numJoints; ++i)
    {
        joint_positions(i)=jointPosition[i];
    }

    // Create the frame that will contain the results

    KDL::Frame cartpos;
    KDL::ChainFkSolverPos_recursive fksolver(kinematicChain);
    KDL::ChainIkSolverVel_pinv iksolver(kinematicChain); //Inverse velocity solver
    KDL::JntArray joint_velocities = KDL::JntArray(kinematicChain.getNrOfJoints());
    iksolver.CartToJnt(joint_positions, temp_twist, joint_velocities);

    return joint_velocities;
}

void RobotHWInterface::newVelCallback(const geometry_msgs::Twist &msg) {
    
    if(mode == TOOL)
    {
        // Calculate indirect kinematics
        jointVelocities = solveIndirectKinematics(msg);
    }
    else
    {
        jointVelocities(0) = msg.linear.x;
        jointVelocities(1) = msg.linear.y;
        jointVelocities(2) = msg.linear.z;
        jointVelocities(3) = msg.angular.x;
        jointVelocities(4) = msg.angular.y;
        jointVelocities(5) = msg.angular.z;
    }

    if(fabs(jointVelocities(i)) < 0.5)
            jointVelocities(i) = 0.0;

    for (int i = 0; i < 6; ++i)
    {
        std_msgs::Float64 velocity;
        velocity.data = jointVelocities(i);
        // commandPub[i].publish(velocity);
    }

}