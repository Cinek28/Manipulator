#include <sstream>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include "RobotHWInterface.h"

using namespace hardware_interface;

RobotHWInterface::RobotHWInterface(ros::NodeHandle &nh) : nodeHandle(nh)
{
    controllerManager.reset(new controller_manager::ControllerManager(this, nodeHandle));
    nodeHandle.param("/manipulator/hardware_interface/rate", rate, 0.1);
    ROS_INFO("Hardware interface update rate: %f", rate);

    KDL::Tree robot_tree;
    std::string robot_desc_string;
    nodeHandle.param("/robot_description", robot_desc_string, std::string());

    if (!kdl_parser::treeFromString(robot_desc_string, robot_tree))
    {
        ROS_ERROR("Failed to construct kdl tree!");
        return;
    }

    robot_tree.getChain("base_link", "gripper_link", kinematicChain);

    KDL::Frame frame(KDL::Rotation::EulerZYX(0.0, -KDL::PI/2, -KDL::PI/2), KDL::Vector(0.0, 0.0, 0.15));
    kinematicChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), frame));

    nodeHandle.param("controller", robot_desc_string, std::string());
    if(robot_desc_string == "position")
        controller = POSITION_CONTROLLER;
    else if(robot_desc_string == "velocity")
        controller = VELOCITY_CONTROLLER;
    
    ROS_INFO("Controller type %s ", robot_desc_string.c_str());

    init();

    jointVelocities = KDL::JntArray(numJoints);
    jointPositions = KDL::JntArray(numJoints);

    if(controller == POSITION_CONTROLLER && mode == TOOL) {
        KDL::Frame tool = solveDirectKinematics(jointPosition);

        jointPositions(0) = tool.p.x();
        jointPositions(1) = tool.p.y();
        jointPositions(2) = tool.p.z();
        tool.M.GetRPY(jointPosition(3), jointPosition(4), jointPosition(5));
    }

    // Sub geometry_msgs::Twist
    twistSub = nodeHandle.subscribe("/spacenav/twist", 1, &RobotHWInterface::newVelCallback, this);

    // Sub geometry_msgs::Twist position
    posSub = nodeHandle.subscribe("/manipulator/position", 1, &RobotHWInterface::newPosCallback, this);

    // Sub trajectory msg:
    trajSub = nodeHandle.subscribe("/manipulator/trajectory", 1, &RobotHWInterface::newTrajCallback, this);

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
    poseStampedPub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/manipulator/tool_pose", 1);

    // Pub visualization marker:
    markerPub = nodeHandle.advertise<visualization_msgs::Marker>("/manipulator/pose_visualization", 1);
    
    markers.header.frame_id = "base_link";
    markers.ns = "manipulator";
    markers.action = visualization_msgs::Marker::ADD;
    markers.type = visualization_msgs::Marker::LINE_STRIP;
    markers.scale.x = 0.005;
    markers.color.g = 1.0f;
    markers.color.a = 1.0;

    realtimeTask = nodeHandle.createWallTimer(ros::WallDuration(1.0 / rate), &RobotHWInterface::update, this);
}

RobotHWInterface::~RobotHWInterface()
{
    for(int i = 0; i < 6; ++i)
        commandPub[i].shutdown();

    commandTrajectoryPub.shutdown();
    jointStateSub.shutdown();
    trajSub.shutdown();
    posSub.shutdown();
    twistSub.shutdown();
    poseStampedPub.shutdown();
    nodeHandle.shutdown();
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
            &jointPosition(i), &jointVelocity(i), &jointEffort(i)));
    }

    registerInterface(&jointStateInt);

    for (int i = 0; i < numJoints; ++i)
    {
        // Create position joint interface
        positionJointInt.registerHandle(
            JointHandle(jointStateInt.getHandle(jointNames[i]),
            &jointPositionCommands(i)));

        //Create velocity joint interface
        velocityJointInt.registerHandle(
            JointHandle(jointStateInt.getHandle(jointNames[i]),
            &jointVelocityCommands(i)));

        // Create effort joint interface
        effortJointInt.registerHandle(
            JointHandle(jointStateInt.getHandle(jointNames[i]),
            &jointEffortCommands(i)));
    }

    for(int i = 0; i < numJoints; ++i)
        ROS_INFO("Joint %d name: %s", i, jointNames[i].c_str());

    registerInterface(&velocityJointInt);
    registerInterface(&positionJointInt);
    registerInterface(&effortJointInt);

    robot.openPort(0, BAUDRATE);
}

void RobotHWInterface::update(const ros::WallTimerEvent &e)
{
    elapsedTime =
        ros::Duration((double)(e.current_real - e.last_real).toNSec()/1000000000);
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
            jointVelocity(i) = (double)(msg.params[i] | (msg.params[i+1] << 8));
        }
    }
}

void RobotHWInterface::write(ros::Duration elapsed_time)
{
    if(mode == TOOL)
    {
        // Calculate indirect kinematics
        if(controller == VELOCITY_CONTROLLER)
            jointVelocityCommands = solveIndirectVelKinematics(jointVelocities);
        else if(controller == POSITION_CONTROLLER) 
            jointPositionCommands = solveIndirectPosKinematics(jointPositions);  
    }
    else
    {
        for(int i = 0; i < numJoints; ++i)
        {
            jointVelocityCommands = jointVelocities;
            jointPositionCommands = jointPositions;
        }
    }

    // Fix for velocity drift in velocity controller:
    if (controller == VELOCITY_CONTROLLER)
    {
        if(jointVelocityCommands(5) == 0.0)
            jointVelocityCommands(5) = 0.0001;

        if(jointVelocityCommands(2) == 0.0)
            jointVelocityCommands(2) = 0.0001;
    }
    
    if(!isRobotConnected() || mode == IDLE)
    {
        std_msgs::Float64 command;

        for(int i = 0; i < numJoints; ++i)
        {
            if(controller == VELOCITY_CONTROLLER)
                command.data = jointVelocityCommands(i);
            else
                command.data = jointPositionCommands(i);

            commandPub[i].publish(command);
        }
    }
    else
    {   
        ManipulatorMsg msg;
        uint16_t velocity = 0;
        msg.type = mode;
        msg.length = 2*numJoints;
        msg.checksum = msg.type + msg.length;
        for (int i = 0; i < numJoints; i++)
        {
            msg.params[i] = (uint8_t)jointVelocityCommands(i);
            msg.params[i+1] = (uint8_t)((uint16_t)jointVelocityCommands(i) >> 8);
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
RobotHWInterface::solveIndirectVelKinematics(const KDL::JntArray &vel)
{
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
        joint_positions(i) = jointPosition(i);
    }

    KDL::ChainFkSolverPos_recursive fksolver(kinematicChain); //Forward position solver
    KDL::ChainIkSolverVel_pinv iksolver(kinematicChain); //Inverse velocity solver
    KDL::JntArray joint_velocities = KDL::JntArray(kinematicChain.getNrOfJoints());

    iksolver.CartToJnt(joint_positions, temp_twist, joint_velocities);

    return joint_velocities;
}

KDL::JntArray
RobotHWInterface::solveIndirectPosKinematics(const KDL::JntArray &pos)
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
        joint_positions(i) = jointPosition(i);
    }

    KDL::ChainFkSolverPos_recursive fksolver(kinematicChain); // Forward pos solver
    KDL::ChainIkSolverVel_pinv iksolver(kinematicChain); // Inverse vel solver
    
    KDL::ChainIkSolverPos_NR iksolver_pos(kinematicChain, fksolver, iksolver);

    KDL::JntArray result = KDL::JntArray(kinematicChain.getNrOfJoints());

    // Calculate inverse position kinematics:
    iksolver_pos.CartToJnt(joint_positions, destination, result);

    return result;
}

void RobotHWInterface::newVelCallback(const geometry_msgs::Twist &msg)
{
    if (isMoving)
    {
        // ROS_INFO("Robot is in moving state. Velocity callback ignored.");
        return;
    }

    jointVelocities(0) = msg.linear.x;
    jointVelocities(1) = msg.linear.y;
    jointVelocities(2) = msg.linear.z;
    jointVelocities(3) = msg.angular.x;
    jointVelocities(4) = msg.angular.y;
    jointVelocities(5) = msg.angular.z;
}

void RobotHWInterface::newPosCallback(const geometry_msgs::Twist &msg)
{
    if (isMoving)
    {
        // ROS_INFO("Robot is in moving state. Position callback ignored.");
        return;
    }

    KDL::JntArray pos(6);
    pos(0) = msg.linear.x;
    pos(1) = msg.linear.y;
    pos(2) = msg.linear.z;
    pos(3) = msg.angular.x;
    pos(4) = msg.angular.y;
    pos(5) = msg.angular.z;

    setManipulatorPos(pos, 1, 6);
}

void RobotHWInterface::newRobotState(const sensor_msgs::JointState &msg)
{
    if(!isRobotConnected() || mode == IDLE)
    {
        jointVelocity(0) = msg.velocity[3];
        jointVelocity(1) = msg.velocity[1];
        jointVelocity(2) = msg.velocity[0];
        jointVelocity(3) = msg.velocity[5];
        jointVelocity(4) = msg.velocity[4];
        jointVelocity(5) = msg.velocity[2];
        jointEffort(0) = msg.effort[3];
        jointEffort(1) = msg.effort[1];
        jointEffort(2) = msg.effort[0];
        jointEffort(3) = msg.effort[5];
        jointEffort(4) = msg.effort[4];
        jointEffort(5) = msg.effort[2];
        jointPosition(0) = msg.position[3];
        jointPosition(1) = msg.position[1];
        jointPosition(2) = msg.position[0];
        jointPosition(3) = msg.position[5];
        jointPosition(4) = msg.position[4];
        jointPosition(5) = msg.position[2];
    }

    KDL::Frame tool = solveDirectKinematics(jointPosition);

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

    geometry_msgs::Point point;
    point.x = tool.p.x();
    point.y = tool.p.y();
    point.z = tool.p.z();
    markers.points.push_back(point);

    markerPub.publish(markers);
}

void RobotHWInterface::newTrajCallback(const manipulator::Trajectory &msg)
{
    if (msg.pos.size() < msg.pointsNr * numJoints)
        ROS_ERROR("Trajectory joint information insufficient (wrong size)");

    ROS_INFO("Generating new trajectory");
    INTERPOLATION_TYPE interp = LINEAR;
    ManipulatorCmd cmdMode = TOOL;

    if(msg.isCubic)
        interp = CUBIC;
    if(!msg.isToolMode)
        cmdMode = JOINT;

    KDL::JntArray pos(msg.pointsNr * numJoints);

    for(int i = 0; i < pos.rows(); ++i)
        pos(i) = msg.pos[i];

    setManipulatorPos(pos, msg.pointsNr, msg.timeSec, interp, msg.interpPoints, cmdMode);
}

bool RobotHWInterface::setVel(const KDL::JntArray &vel)
{
    if(vel.rows() != numJoints)
        return false;

    for(int i = 0; i < numJoints; ++i)
        jointVelocities(i) = vel(i);

    return true;
}

bool RobotHWInterface::setPos(const KDL::JntArray &pos)
{
    if(pos.rows() != numJoints)
        return false;

    for(int i = 0; i < numJoints; ++i)
        jointPositions(i) = pos(i);

    return true;
}

bool RobotHWInterface::setManipulatorPos(const KDL::JntArray &pos,
    size_t pointsNr,
    double timeSec,
    INTERPOLATION_TYPE interp,
    unsigned interpPoints,
    ManipulatorCmd cmdMode)
{
    if(isMoving)
    {
        ROS_INFO("Robot is in moving state. Set cart pos ignored.\n");
        return false;
    }

    isMoving = true;
    mode = cmdMode;

    KDL::JntArray vels = KDL::JntArray(numJoints);
    KDL::JntArray startPos = KDL::JntArray(numJoints);
    KDL::JntArray newPos = KDL::JntArray(numJoints);
    KDL::Frame toolPos;
    KDL::JntArray vini = KDL::JntArray(numJoints);
    KDL::JntArray vfini = KDL::JntArray(numJoints);
    KDL::SetToZero(vini);
    KDL::SetToZero(vfini);

    double startTime = (double)ros::WallTime::now().toNSec() / 1000000000;
    double endTime = startTime + timeSec;
    double currentTime = 0;
    double dT = timeSec / pointsNr;
    double dt = dT / interpPoints;
    double a, b, c, dq = 0;

    ROS_INFO("startTime: %f, endTime: %f\n", startTime, startTime + timeSec);

    markers.action = visualization_msgs::Marker::DELETEALL;
    markerPub.publish(markers);
    markers.points.clear();
    markers.action = visualization_msgs::Marker::ADD;

    for(int i = 0; i < pointsNr; ++i)
    {
        if(mode == TOOL) {            
            toolPos = solveDirectKinematics(jointPosition);
            startPos(0) = toolPos.p.x();
            startPos(1) = toolPos.p.y();
            startPos(2) = toolPos.p.z();
            toolPos.M.GetRPY(startPos(3), startPos(4), startPos(5));
        }

        ROS_INFO("Point %d:", i);
        
        for(int j = 0; j < numJoints; ++j)
            vini(j) = vfini(j);

        if(interp == LINEAR)
        {
            for(int j = i * numJoints; j < (i + 1) * numJoints; ++j)
            {
                dq = pos(j) - startPos(j % numJoints);
                vels(j % numJoints) = dq / dT;
            }
        }

        for(int j = 0; j < interpPoints; ++j)
        {
            ROS_INFO("Point[%d][%d]:", i, j);

            for(int k = 0; k < numJoints; ++k)
                ROS_INFO("[%d]: %f, ", k, startPos(k) + vels(k) * (j+1) * dt);

            endTime = (j + 1) * dt;

            while(currentTime < endTime)
            {
                currentTime = (double)ros::WallTime::now().toNSec() / 1000000000;

                if(currentTime >= startTime + timeSec)
                {
                    ROS_INFO("TIMEOUT!");
                    i = pointsNr;
                    j = interpPoints;
                    break;
                }
                
                currentTime -= (startTime + i * dT);

                for(int k = 0; k < numJoints; ++k)
                {
                    if(interp == LINEAR)
                    {
                        newPos(k) = startPos(k) + vels(k) * currentTime;
                    }
                    else
                    {
                        dq = pos(i * numJoints + k) - startPos(k);
                        vfini(k) = -0.5 * vini(k) + 1.5 * dq / dT;
                        a = -0.5 + 1.5 * vini(k) * dT / dq; 
                        b = 1.5 - 2.5 * vini(k) * dT / dq;
                        c = vini(k) * dT / dq;

                        vels(k) = dq * (3 * a * pow(currentTime, 2.0) / pow(dT, 3.0)
                            + 2 * b * currentTime / pow(dT, 2.0) + c / dT);

                        newPos(k) = startPos(k) + dq * (a * pow(currentTime / dT, 3.0) +
                            b * pow(currentTime / dT, 2.0) + c * currentTime / dT);
                    }
                }

                setVel(vels);
                setPos(newPos);
            }

            currentTime = 0;
        }
    }

    ROS_INFO("Ending trajectory, duration: %f\n",
        (double)ros::WallTime::now().toNSec() / 1000000000 - startTime);

    toolPos = solveDirectKinematics(jointPosition);
    startPos(0) = toolPos.p.x();
    startPos(1) = toolPos.p.y();
    startPos(2) = toolPos.p.z();
    toolPos.M.GetRPY(startPos(3), startPos(4), startPos(5));

    ROS_INFO("Ending position:\t");

    for(int i = 0; i < numJoints; ++i) 
        ROS_INFO("[%d]: %f, ", i, startPos(i));

    KDL::SetToZero(vels);
    setVel(vels);
    isMoving = false;

    return true;
}

bool RobotHWInterface::setCartPos(ros::NodeHandle &nh,
    const std::vector<double> &pos,
    size_t pointsNr,
    double timeSec,
    bool isCubic,
    unsigned interpPoints)
{
    manipulator::Trajectory traj;
    traj.pos = pos;
    traj.pointsNr = pointsNr;
    traj.timeSec = timeSec;
    traj.isCubic = isCubic;
    traj.interpPoints = interpPoints;
    traj.isToolMode = true;

    ros::Publisher pub = nh.advertise<manipulator::Trajectory>("/manipulator/trajectory", 1, true);

    pub.publish(traj);

    ros::spinOnce();
    ros::Duration(timeSec).sleep();

    return true;
}

bool RobotHWInterface::setJntPos(ros::NodeHandle &nh,
    const std::vector<double> &pos,
    size_t pointsNr,
    double timeSec,
    bool isCubic,
    unsigned interpPoints)
{
    manipulator::Trajectory traj;
    traj.pos = pos;
    traj.pointsNr = pointsNr;
    traj.timeSec = timeSec;
    traj.isCubic = isCubic;
    traj.interpPoints = interpPoints;
    traj.isToolMode = false;

    ros::Publisher pub = nh.advertise<manipulator::Trajectory>("/manipulator/trajectory", 1, true);

    pub.publish(traj);

    ros::spinOnce();
    ros::Duration(timeSec).sleep();

    return true;
}