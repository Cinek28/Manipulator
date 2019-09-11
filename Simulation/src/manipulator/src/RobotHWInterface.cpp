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

    // Sub sensor_msgs::JointState
    jointStateSub = nodeHandle.subscribe("/manipulator/joint_states",1,&RobotHWInterface::newPosCallback, this);

    // Pub external controller steering commands:
    commandPub[0] = nodeHandle.advertise<std_msgs::Float64>("/manipulator/shoulder_rotation_controller/command", 1);
    commandPub[1] = nodeHandle.advertise<std_msgs::Float64>("/manipulator/forearm_rotation_controller/command", 1);
    commandPub[2] = nodeHandle.advertise<std_msgs::Float64>("/manipulator/arm_rotation_controller/command", 1);
    commandPub[3] = nodeHandle.advertise<std_msgs::Float64>("/manipulator/wrist_rotation_controller/command", 1);
    commandPub[4] = nodeHandle.advertise<std_msgs::Float64>("/manipulator/wrist_pitch_controller/command", 1);
    commandPub[5] = nodeHandle.advertise<std_msgs::Float64>("/manipulator/gripper_rotation_controller/command", 1);

    // Pub external controller trajectory commands:
    commandTrajectoryPub = nodeHandle.advertise<trajectory_msgs::JointTrajectory>
        ("/manipulator/joint_trajectory_controller/command",1);

    std_msgs::Float64 initial_vel;
    initial_vel.data = 0.0;

    for(int i = 0; i < numJoints; ++i)
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
            velocityJointInt.getHandle(jointNames[i]).setCommand(jointVelocities(i));
            std_msgs::Float64 velocity;
            velocity.data = jointVelocities(i);
            commandPub[i].publish(velocity);
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
            msg.params[i] = (uint8_t)jointVelocities(i);
            msg.params[i+1] = (uint8_t)((uint16_t)jointVelocities(i) >> 8);
            msg.checksum += msg.params[i] + msg.params[i+1];
        }
        msg.checksum = ~msg.checksum;
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

    KDL::ChainFkSolverPos_recursive fksolver(kinematicChain); //Forward position solver
    KDL::ChainIkSolverVel_pinv iksolver(kinematicChain); //Inverse velocity solver
    KDL::JntArray joint_velocities = KDL::JntArray(kinematicChain.getNrOfJoints());

    iksolver.CartToJnt(joint_positions, temp_twist, joint_velocities);

    return joint_velocities;
}

KDL::JntArray RobotHWInterface::solveIndirectPositionKinematics
    (const geometry_msgs::Twist &msg)
{
    // Create destination frame
    KDL::Frame destination = 
        KDL::Frame(KDL::Rotation::RPY(msg.angular.x, msg.angular.y, msg.angular.z),
                   KDL::Vector(msg.linear.x, msg.linear.y, msg.linear.z));

    // Create joint array
    KDL::JntArray joint_positions = KDL::JntArray(kinematicChain.getNrOfJoints());
    for(int i = 0; i < numJoints; ++i)
    {
        joint_positions(i)=jointPosition[i];
    }

    KDL::ChainFkSolverPos_recursive fksolver(kinematicChain); // Forward pos solver
    KDL::ChainIkSolverVel_pinv iksolver(kinematicChain); // Inverse vel solver
    
    ChainIkSolverPos_NR iksolver_pos(kinematicChain,fksolver,iksolver,100,1e-6);

    KDL::JntArray result = KDL::JntArray(kinematicChain.getNrOfJoints());

    // Calculate inverse position kinematics:
    iksolver_pos.CartToJnt(joint_positions, destination, result);
}

void RobotHWInterface::newVelCallback(const geometry_msgs::Twist &msg)
{
    geometry_msgs::Twist converted_msg = msg;
    if(fabs(converted_msg.linear.x) < 0.5)
        converted_msg.linear.x = 0;
    if(fabs(converted_msg.linear.y) < 0.5)
        converted_msg.linear.y = 0;
    if(fabs(converted_msg.linear.z) < 0.5)
        converted_msg.linear.z = 0;
    if(fabs(converted_msg.angular.x) < 0.5)
        converted_msg.angular.x = 0;
    if(fabs(converted_msg.angular.y) < 0.5)
        converted_msg.angular.y = 0;
    if(fabs(converted_msg.linear.x) < 0.5)
        converted_msg.angular.z = 0;

    if(mode == TOOL)
    {
        // Calculate indirect kinematics
        jointVelocities = solveIndirectKinematics(converted_msg);
    }
    else {
        jointVelocities(0) = converted_msg.linear.x;
        jointVelocities(1) = converted_msg.linear.y;
        jointVelocities(2) = converted_msg.linear.z;
        jointVelocities(3) = converted_msg.angular.x;
        jointVelocities(4) = converted_msg.angular.y;
        jointVelocities(5) = converted_msg.angular.z;
    }
}

void RobotHWInterface::newPosCallback(const sensor_msgs::JointState& msg)
{
    for(int i = 0; i < numJoints; ++i)
    {
        if(!isRobotConnected() || mode == IDLE)
        {
            jointVelocity[i] = msg.velocity[i];
            jointPosition[i] = msg.position[i];
            jointEffort[i] = msg.effort[i];
        }
    }
}

void RobotHWInterface::newTrajCallback(const sensor_msgs::JointState &msg)
{

    KDL::JntArray trajPoint = solveIndirectPositionKinematics(msg);

    trajectory_msgs::JointTrajectory traj;

    traj.header.frame_id = "base_link";
    traj.joint_names.resize(numJoints);
    traj.points.resize(1);

    traj.points[0].positions.resize(numJoints);

    traj.joint_names[0] = "shoulder_rotation";
    traj.joint_names[1] = "forearm_rotation";
    traj.joint_names[2] = "arm_rotation";
    traj.joint_names[3] = "wrist_pitch";
    traj.joint_names[4] = "wrist_rotation";
    traj.joint_names[5] = "gripper_rotation";

    traj.header.stamp = ros::Time::now();
    traj.points[0].time_from_start = ros::Duration(1);

    for(int i = 0; i < numJoints; ++i)
        traj.points[0].positions[i] = trajPoint(i);

    commandTrajectoryPub.publish(traj);
}