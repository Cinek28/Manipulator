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
    controllerManager.reset(new controller_manager::ControllerManager(this, nodeHandle));
    nodeHandle.param("/manipulator/hardware_interface/rate", rate, 0.1);

    KDL::Tree robot_tree;
    std::string robot_desc_string;
    nodeHandle.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, robot_tree))
    {
        ROS_ERROR("Failed to construct kdl tree!");
        return;
    }

    robot_tree.getChain("base_link", "gripper_link", kinematicChain);

//    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Vector(0, 0, 0.13))));

//    KDL::Frame frame = KDL::Frame(KDL::Rotation::RPY(KDL::PI/2,0.0,0.0),KDL::Vector(0, 0.00175, 0.0705));
//    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));
//
//    frame = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,-1.283),KDL::Vector(0, 0.6, 0.0));
//    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

//    frame = KDL::Frame(KDL::Rotation::EulerZYX(0.0,-KDL::PI/2,0.0),KDL::Vector(0.408, 0.005, 0.0));
//    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));
//
//    frame = KDL::Frame(KDL::Rotation::EulerZYX(-KDL::PI/2,0.0,3*KDL::PI/2),KDL::Vector(0.0, 0.0, -0.129));
//    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));
//
//    frame = KDL::Frame(KDL::Rotation::EulerZYX(0.0,-KDL::PI/2,0.0),KDL::Vector(0.0643, 0.0, 0.0));
//    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));
//
//    frame = KDL::Frame(KDL::Rotation::EulerZYX(0.0,KDL::PI/2,0.0),KDL::Vector(0.0, 0.0, -0.15));
//    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    init();

    jointVelocities = KDL::JntArray(numJoints);

    // Sub geometry_msgs::Twist
    twistSub = nodeHandle.subscribe("/spacenav/twist",1,&RobotHWInterface::newVelCallback, this);

    // Sub sensor_msgs::JointState
    jointStateSub = nodeHandle.subscribe("/manipulator/joint_states",1,&RobotHWInterface::newPosCallback, this);

    // Sub trajectory msg:
    trajSub = nodeHandle. subscribe("/manipulator/new_point_trajectory",1,&RobotHWInterface::newTrajCallback, this);

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

    // Pub direct kinematic solved pose:
    poseStampedPub = nodeHandle.advertise<geometry_msgs::PoseStamped>
            ("/manipulator/tool_pose", 1);

    std_msgs::Float64 initial_vel;
    initial_vel.data = 0.0;

    for(int i = 0; i < numJoints; ++i)
        commandPub[i].publish(initial_vel);

    nonRealtimeTask = nodeHandle.createTimer(ros::Duration(1.0 / rate),
                                             &RobotHWInterface::update, this);
}

RobotHWInterface::~RobotHWInterface()
{
}

void RobotHWInterface::init()
{
    // Get joint names
    nodeHandle.getParam("/manipulator/hardware_interface/joints", jointNames);
    numJoints = kinematicChain.getNrOfJoints();
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

    for(int i = 0; i < numJoints; ++i)
        ROS_INFO("Joint %d name: %s", i, jointNames[i].c_str());

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

geometry_msgs::PoseStamped
RobotHWInterface::solveDirectKinematics(const sensor_msgs::JointState &msg)
{
    //Create joint array
    KDL::JntArray joint_positions =
            KDL::JntArray(kinematicChain.getNrOfJoints());

    joint_positions(0) = msg.position[3];
    joint_positions(1) = msg.position[1];
    joint_positions(2) = msg.position[0];
    joint_positions(3) = msg.position[5];
    joint_positions(4) = msg.position[4];
    joint_positions(5) = msg.position[2];

    // Create the frame that will contain the results
    KDL::Frame cartpos;
    KDL::ChainFkSolverPos_recursive fksolver(kinematicChain);
    // Calculate forward position kinematics
    fksolver.JntToCart(joint_positions,cartpos);

    // Fill the position part
    geometry_msgs::PoseStamped tool_pose;

    tool_pose.pose.position.x = cartpos.p.x();
    tool_pose.pose.position.y = cartpos.p.y();
    tool_pose.pose.position.z = cartpos.p.z();

    cartpos.M.GetQuaternion(tool_pose.pose.orientation.x,
                            tool_pose.pose.orientation.y,
                            tool_pose.pose.orientation.z,
                            tool_pose.pose.orientation.w);

    // Transformation is happening in base_link
    tool_pose.header.frame_id = "base_link";

    return tool_pose;
}

KDL::JntArray
RobotHWInterface::solveIndirectVelKinematics(
        const geometry_msgs::Twist &msg) {

    KDL::Twist temp_twist;
    temp_twist.vel.x(msg.linear.x);
    temp_twist.vel.y(msg.linear.y);
    temp_twist.vel.z(msg.linear.z);
    temp_twist.rot.x(msg.angular.x);
    temp_twist.rot.y(msg.angular.y);
    temp_twist.rot.z(msg.angular.z);

    //Create joint array
    KDL::JntArray joint_positions =
            KDL::JntArray(kinematicChain.getNrOfJoints());
    for(int i = 0; i < numJoints; ++i)
    {
        joint_positions(i) = jointPosition[i];
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
    KDL::JntArray joint_positions =
            KDL::JntArray(kinematicChain.getNrOfJoints());
    for(int i = 0; i < kinematicChain.getNrOfJoints(); ++i)
    {
        joint_positions(i)=jointPosition[i];
    }

    KDL::ChainFkSolverPos_recursive fksolver(kinematicChain); // Forward pos solver
    KDL::ChainIkSolverVel_pinv iksolver(kinematicChain); // Inverse vel solver
    
    KDL::ChainIkSolverPos_NR iksolver_pos(kinematicChain,fksolver,iksolver,100,1e-6);

    KDL::JntArray result = KDL::JntArray(kinematicChain.getNrOfJoints());

    // Calculate inverse position kinematics:
    iksolver_pos.CartToJnt(joint_positions, destination, result);

    return result;
}

void RobotHWInterface::newVelCallback(const geometry_msgs::Twist &msg)
{
    geometry_msgs::Twist converted_msg = msg;
//    if(fabs(converted_msg.linear.x) < 0.5)
//        converted_msg.linear.x = 0;
//    if(fabs(converted_msg.linear.y) < 0.5)
//        converted_msg.linear.y = 0;
//    if(fabs(converted_msg.linear.z) < 0.5)
//        converted_msg.linear.z = 0;
//    if(fabs(converted_msg.angular.x) < 0.5)
//        converted_msg.angular.x = 0;
//    if(fabs(converted_msg.angular.y) < 0.5)
//        converted_msg.angular.y = 0;
//    if(fabs(converted_msg.linear.x) < 0.5)
//        converted_msg.angular.z = 0;

    if(mode == TOOL)
    {
        // Calculate indirect kinematics
        jointVelocities = solveIndirectVelKinematics(converted_msg);
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

void RobotHWInterface::newPosCallback(const sensor_msgs::JointState &msg)
{
//    for(int i = 0; i < numJoints; ++i)
//    {
        if(!isRobotConnected() || mode == IDLE)
        {
            jointVelocity[0] = msg.velocity[3];
            jointVelocity[1] = msg.velocity[1];
            jointVelocity[2] = msg.velocity[0];
            jointVelocity[3] = msg.velocity[5];
            jointVelocity[4] = msg.velocity[4];
            jointVelocity[5] = msg.velocity[2];
            jointEffort[0] = msg.effort[3];
            jointEffort[1] = msg.effort[1];
            jointEffort[2] = msg.effort[0];
            jointEffort[3] = msg.effort[5];
            jointEffort[4] = msg.effort[4];
            jointEffort[5] = msg.effort[2];
            jointPosition[0] = msg.position[3];
            jointPosition[1] = msg.position[1];
            jointPosition[2] = msg.position[0];
            jointPosition[3] = msg.position[5];
            jointPosition[4] = msg.position[4];
            jointPosition[5] = msg.position[2];
        }
//    }

    geometry_msgs::PoseStamped tool_pose = solveDirectKinematics(msg);
    poseStampedPub.publish(tool_pose);

}

void RobotHWInterface::newTrajCallback(const geometry_msgs::Twist &msg)
{

    KDL::JntArray trajPoint = solveIndirectPositionKinematics(msg);

    trajectory_msgs::JointTrajectory traj;

    traj.header.frame_id = "base_link";
    traj.joint_names.resize(numJoints);
    traj.points.resize(1);

    traj.points[0].positions.resize(numJoints);

    for(int i = 0; i < numJoints; ++i)
        traj.joint_names[i] = jointNames[i];

    traj.header.stamp = ros::Time::now();
    traj.points[0].time_from_start = ros::Duration(10);

    for(int i = 0; i < numJoints; ++i)
        traj.points[0].positions[i] = trajPoint(i);

    ROS_INFO("Generating trajectory");

    commandTrajectoryPub.publish(traj);
}