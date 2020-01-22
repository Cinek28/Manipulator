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

    init();

    jointVelocities = KDL::JntArray(numJoints);
    jointPositions = KDL::JntArray(numJoints);
    jointCommand = KDL::JntArray(numJoints);

    // Sub geometry_msgs::Twist
    twistSub = nodeHandle.subscribe("/spacenav/twist", 1, &RobotHWInterface::newVelCallback, this);

    // Sub geometry_msgs::Twist position
    posSub = nodeHandle.subscribe("/manipulator/position", 1, &RobotHWInterface::newPosCallback, this);

    // Sub trajectory msg:
    trajSub = nodeHandle. subscribe("/manipulator/new_point_trajectory", 1, &RobotHWInterface::newTrajCallback, this);

    // Sub sensor_msgs::JointState
    jointStateSub = nodeHandle.subscribe("/manipulator/joint_states", 1, &RobotHWInterface::newRobotState, this);

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
        jointStateInt.registerHandle(JointStateHandle(jointNames[i],
            &jointPosition[i], &jointVelocity[i], &jointEffort[i]));

        // Create position joint interface
//        JointLimits limits;
//        SoftJointLimits softLimits;
//        getJointLimits(jointNames[i], nodeHandle, limits);
//        PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
//        positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
        positionJointInt.registerHandle(
            JointHandle(jointStateInt.getHandle(jointNames[i]),
            &jointPositionCommands[i]));

        //Create velocity joint interface:
        velocityJointInt.registerHandle(
            JointHandle(jointStateInt.getHandle(jointNames[i]),
            &jointVelocityCommands[i]));

        // Create effort joint interface
        effortJointInt.registerHandle(
            JointHandle(jointStateInt.getHandle(jointNames[i]),
            &jointEffortCommands[i]));
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
    if(mode == TOOL)
    {
        // Calculate indirect kinematics
        if(controller == VELOCITY_CONTROLLER)
            jointCommand = solveIndirectVelKinematics(jointVelocities);
        else
            jointCommand = solveIndirectPosKinematics(jointPositions);  
    }

    if(!isRobotConnected() || mode == IDLE)
    {
        for(int i = 0; i < numJoints; ++i)
        {
            if(controller == VELOCITY_CONTROLLER) {
                velocityJointInt.getHandle(jointNames[i]).setCommand(jointCommand(i));
            }
            else {
                positionJointInt.getHandle(jointNames[i]).setCommand(jointCommand(i)); 
            }

            std_msgs::Float64 command;
            command.data = jointCommand(i);
            commandPub[i].publish(command);
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
            msg.params[i] = (uint8_t)jointCommand(i);
            msg.params[i+1] = (uint8_t)((uint16_t)jointCommand(i) >> 8);
            msg.checksum += msg.params[i] + msg.params[i+1];
        }
        msg.checksum = ~msg.checksum;
        robot.sendData(&msg);
    }
}

KDL::Frame
RobotHWInterface::solveDirectKinematics(const KDL::JntArray &pos)
{
    // Create the frame that will contain the results
    KDL::Frame cartpos;
    KDL::ChainFkSolverPos_recursive fksolver(kinematicChain);
    // Calculate forward position kinematics
    fksolver.JntToCart(pos,cartpos);

    return cartpos;
}

KDL::JntArray
RobotHWInterface::solveIndirectVelKinematics(
        const KDL::JntArray &vel) {

    KDL::Twist temp_twist;
    temp_twist.vel.x(vel(0));
    temp_twist.vel.y(vel(1));
    temp_twist.vel.z(vel(2));
    temp_twist.rot.x(vel(3));
    temp_twist.rot.y(vel(4));
    temp_twist.rot.z(vel(5));

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

KDL::JntArray RobotHWInterface::solveIndirectPosKinematics
    (const KDL::JntArray &pos)
{
    // Create destination frame
    KDL::Frame destination = 
        KDL::Frame(KDL::Rotation::RPY(pos(3), pos(4), pos(5)),
                   KDL::Vector(pos(0), pos(1), pos(2)));

    // Create joint array
    KDL::JntArray joint_positions =
            KDL::JntArray(kinematicChain.getNrOfJoints());
            
    for(int i = 0; i < kinematicChain.getNrOfJoints(); ++i)
    {
        joint_positions(i) = jointPosition[i];
    }

    KDL::ChainFkSolverPos_recursive fksolver(kinematicChain); // Forward pos solver
    KDL::ChainIkSolverVel_pinv iksolver(kinematicChain); // Inverse vel solver
    
    KDL::ChainIkSolverPos_NR iksolver_pos(kinematicChain, fksolver, iksolver, 100,1e-6);

    KDL::JntArray result = KDL::JntArray(kinematicChain.getNrOfJoints());

    // Calculate inverse position kinematics:
    iksolver_pos.CartToJnt(joint_positions, destination, result);

    return result;
}

void RobotHWInterface::newVelCallback(const geometry_msgs::Twist &msg)
{
    if (isMoving)
    {
        ROS_INFO("Robot is in moving state. Velocity callback ignored.");
        return;
    }

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

    jointVelocities(0) = converted_msg.linear.x;
    jointVelocities(1) = converted_msg.linear.y;
    jointVelocities(2) = converted_msg.linear.z;
    jointVelocities(3) = converted_msg.angular.x;
    jointVelocities(4) = converted_msg.angular.y;
    jointVelocities(5) = converted_msg.angular.z;
}

void RobotHWInterface::newPosCallback(const geometry_msgs::Twist &msg)
{
    if (isMoving)
    {
        ROS_INFO("Robot is in moving state. Position callback ignored.");
        return;
    }

    KDL::JntArray pos(6);
    pos(0) = msg.linear.x;
    pos(1) = msg.linear.y;
    pos(2) = msg.linear.z;
    pos(3) = msg.angular.x;
    pos(4) = msg.angular.y;
    pos(5) = msg.angular.z;

    setCartPos(&pos, 1, 6);
}

void RobotHWInterface::newRobotState(const sensor_msgs::JointState &msg)
{
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

    KDL::JntArray pos(numJoints);
    for(int i = 0; i < numJoints; ++i)
        pos(i) = jointPosition[i];

    KDL::Frame tool = solveDirectKinematics(pos);

    // Fill the position part
    geometry_msgs::PoseStamped tool_pose;

    tool_pose.pose.position.x = tool.p.x();
    tool_pose.pose.position.y = tool.p.y();
    tool_pose.pose.position.z = tool.p.z();

    tool.M.GetQuaternion(tool_pose.pose.orientation.x,
                            tool_pose.pose.orientation.y,
                            tool_pose.pose.orientation.z,
                            tool_pose.pose.orientation.w);

    // Transformation is happening in base_link
    tool_pose.header.frame_id = "base_link";
    poseStampedPub.publish(tool_pose);
}

void RobotHWInterface::newTrajCallback(const geometry_msgs::Twist &msg)
{
    KDL::JntArray trajCartPoint = KDL::JntArray(numJoints);
    trajCartPoint(0) = msg.linear.x;
    trajCartPoint(1) = msg.linear.y;
    trajCartPoint(2) = msg.linear.z;
    trajCartPoint(3) = msg.angular.x;
    trajCartPoint(4) = msg.angular.y;
    trajCartPoint(5) = msg.angular.z;

    KDL::JntArray trajJntPoint = solveIndirectPosKinematics(trajCartPoint);

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
        traj.points[0].positions[i] = trajJntPoint(i);

    ROS_INFO("Generating trajectory");

    commandTrajectoryPub.publish(traj);
}

bool RobotHWInterface::setVel(const KDL::JntArray &vel)
{
    if(vel.rows() != numJoints)
        return false;

    jointVelocities = vel;

    return true;
}

bool RobotHWInterface::setCartPos(const KDL::JntArray *pos,
    size_t pointsNr,
    double timeSec,
    INTERPOLATION_TYPE interp,
    unsigned interpPoints)
{
    if(isMoving)
    {
        ROS_INFO("Robot is in moving state. Set cart pos ignored.\n");
        return false;
    }

    isMoving = true;
    mode = TOOL;
    controller = VELOCITY_CONTROLLER;

    KDL::JntArray vel = KDL::JntArray(numJoints);
    KDL::JntArray newPos = KDL::JntArray(numJoints);
    KDL::Frame toolPos;
    double startTime = ros::Time::now().toNSec() / 1000000000;
    double endTime = startTime + timeSec / interpPoints;
    ROS_INFO("startTime: %f, endTime: %f\n", startTime, endTime);

    if(interp == LINEAR)
    {
        for(int i = 0; i < pointsNr; ++i)
        {
            ROS_INFO("Point %d\n", i);
            if(pos[i].rows() != numJoints)
            {
                ROS_INFO("Wrong position array size %d\n", i);
                KDL::SetToZero(vel);
                setVel(vel);
                isMoving = false;
                return false;
            }

            for(int j = 0; j < numJoints; ++j)
                newPos(j) = jointPosition[j];

            toolPos = solveDirectKinematics(newPos);
            newPos(0) = toolPos.p.x();
            newPos(1) = toolPos.p.y();
            newPos(2) = toolPos.p.z();
            toolPos.M.GetRPY(newPos(3), newPos(4), newPos(5));

            for(int j = 0; j < numJoints; ++j)
            {
                vel(j) = (pos[i](j) - newPos(j)) / timeSec;
            }

            for(int j = 0; j < interpPoints; ++j)
            {
                ROS_INFO("Interp Point %d %d\n", i, j);
                setVel(vel);
                while(ros::Time::now().toNSec() / 1000000000 < endTime);
                endTime += timeSec / interpPoints;
            }
        }
    }
    else
    {
        for(int i = 0; i < pointsNr; ++i)
        {
            ROS_INFO("Point %d\n", i);
            if(pos[i].rows() != numJoints)
            {
                ROS_INFO("Wrong position array size %d\n", i);
                KDL::SetToZero(vel);
                setVel(vel);
                isMoving = false;
                return false;
            }

            for(int j = 0; j < numJoints; ++j)
                newPos(j) = jointPosition[j];

            toolPos = solveDirectKinematics(newPos);
            newPos(0) = toolPos.p.x();
            newPos(1) = toolPos.p.y();
            newPos(2) = toolPos.p.z();
            toolPos.M.GetRPY(newPos(3), newPos(4), newPos(5));

            for(int j = 0; j < numJoints; ++j)
            {
                vel(j) = (pos[i](j) - newPos(j)) / timeSec;
            }

            for(int j = 0; j < interpPoints; ++j)
            {
                ROS_INFO("Interp Point %d %d\n", i, j);
                setVel(vel);
                while(ros::Time::now().toNSec() / 1000000000 < endTime) 
                    ros::spinOnce();
                endTime += timeSec / interpPoints;
            }
        }
        ROS_INFO("Cubic interpolation not implemented\n");
        isMoving = false;
        return false;
    }

    ROS_INFO("Ending trajectory\n");
    KDL::SetToZero(vel);
    setVel(vel);
    isMoving = false;

    return true;
}

bool RobotHWInterface::setJntPos(const KDL::JntArray &pos,
    size_t pointsNr,
    double timeSec,
    INTERPOLATION_TYPE interp,
    unsigned interpPoints)
{
    if(isMoving)
    {
        ROS_INFO("Robot is in moving state. Set jnt pos ignored.");
        return false;
    }

    isMoving = true;
    mode = TOOL;
    controller = VELOCITY_CONTROLLER;

    isMoving = false;

    return false;
}