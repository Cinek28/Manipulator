#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>


class KDLSolver {
public:
    KDLSolver();
    void newStateCallback(const sensor_msgs::JointState &msg);
    ros::Time time;
    static const unsigned int JOINTS_AMOUNT = 6;

private:
    geometry_msgs::PoseStamped solveDirectKinematics(const sensor_msgs::JointState &msg);

    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber twist_one_sub_;
    ros::Publisher pose_stamped_pub_;
    ros::Publisher joint_state_pub_;
    sensor_msgs::JointState new_state;

    double theta_[JOINTS_AMOUNT] = {0.0};
    double theta_limit_lower_[JOINTS_AMOUNT] = {-3.14};
    double theta_limit_upper_[JOINTS_AMOUNT] = {3.14};
    bool is_d_kin_pose_reachable_;

    KDL::Chain chain;
};

KDLSolver::KDLSolver(){
    bool is_limit_param_ok = true;
    is_limit_param_ok = nh_.param("joint1/limit/lower", theta_limit_lower_[0], theta_limit_lower_[0]);
    is_limit_param_ok = nh_.param("joint1/limit/upper", theta_limit_upper_[0], theta_limit_upper_[0]);
    is_limit_param_ok = nh_.param("joint2/limit/lower", theta_limit_lower_[1], theta_limit_lower_[1]);
    is_limit_param_ok = nh_.param("joint2/limit/upper", theta_limit_upper_[1], theta_limit_upper_[1]);
    is_limit_param_ok = nh_.param("joint3/limit/lower", theta_limit_lower_[2], theta_limit_lower_[2]);
    is_limit_param_ok = nh_.param("joint3/limit/upper", theta_limit_upper_[2], theta_limit_upper_[2]);
    is_limit_param_ok = nh_.param("joint4/limit/lower", theta_limit_lower_[3], theta_limit_lower_[3]);
    is_limit_param_ok = nh_.param("joint4/limit/upper", theta_limit_upper_[3], theta_limit_upper_[3]);
    is_limit_param_ok = nh_.param("joint5/limit/lower", theta_limit_lower_[4], theta_limit_lower_[4]);
    is_limit_param_ok = nh_.param("joint5/limit/upper", theta_limit_upper_[4], theta_limit_upper_[4]);
    is_limit_param_ok = nh_.param("joint6/limit/lower", theta_limit_lower_[5], theta_limit_lower_[5]);
    is_limit_param_ok = nh_.param("joint6/limit/upper", theta_limit_upper_[5], theta_limit_upper_[5]);

    if (is_limit_param_ok == false) {
        ROS_WARN("Something wrong with getting at least one limit param. Using default value(s)");
    }

    // Sub sensor_msgs/JointState
    joint_state_sub_ = nh_.subscribe("/joint_states",1, &KDLSolver::newStateCallback, this);

    // Pub geometry_msgs/PoseStamped
    pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/kdl_dkin", 1);

    KDL::Tree robot_tree;
    std::string robot_desc_string;
    nh_.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, robot_tree))
    {
        ROS_INFO("%s", robot_desc_string.c_str());
        ROS_ERROR("Failed to construct kdl tree!");
        return;
    }

    robot_tree.getChain("base_link", "arm_link", chain);

//    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Vector(0, 0, 0.13))));
//
//    KDL::Frame frame = KDL::Frame(KDL::Rotation::RPY(-KDL::PI/2,0.0,0.0),KDL::Vector(0, 0.00175, 0.0705));
//    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));
//
//    frame = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,1.283),KDL::Vector(0, -0.6, 0.0));
//    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));
//
//    frame = KDL::Frame(KDL::Rotation::EulerZYX(0.0,-KDL::PI/2,0.0),KDL::Vector(0.408, 0.005, 0.0));
//    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));
//
//    frame = KDL::Frame(KDL::Rotation::EulerZYX(-KDL::PI/2,0.0,3*KDL::PI/2),KDL::Vector(0.0, 0.0, -0.129));
//    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));
//
//    frame = KDL::Frame(KDL::Rotation::EulerZYX(0.0,-KDL::PI/2,0.0),KDL::Vector(0.0643, 0.0, 0.0));
//    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));
//
//    frame = KDL::Frame(KDL::Rotation::EulerZYX(0.0,KDL::PI/2,0.0),KDL::Vector(0.0, 0.0, -0.15));
//    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    new_state.name.push_back("shoulder_rotation");
    new_state.name.push_back("forearm_rotation");
    new_state.name.push_back("arm_rotation");
    new_state.name.push_back("wrist_rotation");
    new_state.name.push_back("wrist_pitch");
    new_state.name.push_back("gripper_rotation");
    new_state.position.push_back(0.0);
    new_state.position.push_back(0.0);
    new_state.position.push_back(0.0);
    new_state.position.push_back(0.0);
    new_state.position.push_back(0.0);
    new_state.position.push_back(0.0);
};

geometry_msgs::PoseStamped KDLSolver::solveDirectKinematics(const sensor_msgs::JointState &msg) {
    is_d_kin_pose_reachable_ = true;

    for (unsigned int i = 0; i < JOINTS_AMOUNT; i++) {
        theta_[i] = msg.position.at(i);
        if ((theta_[i] < theta_limit_lower_[i]) ||
            (theta_[i] > theta_limit_upper_[i])) {
            is_d_kin_pose_reachable_ = false;
            ROS_WARN("Reached joint %d limit! Not moving further!", (i + 1));
        }
    }

    //Create joint array
    KDL::JntArray jointpositions = KDL::JntArray(chain.getNrOfJoints());
    jointpositions(0)=theta_[0];
    jointpositions(1)=theta_[1];
    jointpositions(2)=theta_[2];
    jointpositions(3)=theta_[3];
    jointpositions(4)=theta_[4];
    jointpositions(5)=theta_[5];
    // Create the frame that will contain the results

    KDL::Frame cartpos;
    KDL::ChainFkSolverPos_recursive fksolver(chain);
    // Calculate forward position kinematics
    fksolver.JntToCart(jointpositions,cartpos);

    // Fill the position part
    geometry_msgs::PoseStamped temp_pose;

    temp_pose.pose.position.x = cartpos.p.x();
    temp_pose.pose.position.y = cartpos.p.y();
    temp_pose.pose.position.z = cartpos.p.z();

    cartpos.M.GetQuaternion(temp_pose.pose.orientation.x, temp_pose.pose.orientation.y,
                            temp_pose.pose.orientation.z, temp_pose.pose.orientation.w);

    // Transformation is happening in base_link
    temp_pose.header.frame_id = "base_link";

    return temp_pose;
}

void KDLSolver::newStateCallback(const sensor_msgs::JointState &msg) {
    // Calculate direct kinematics
    geometry_msgs::PoseStamped d_kin_pose;

    d_kin_pose = solveDirectKinematics(msg);

    if (is_d_kin_pose_reachable_) {
        // Publish calculated position
        pose_stamped_pub_.publish(d_kin_pose);
    }
}


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "kdl_dkin");
    KDLSolver kinematics_solver;

    puts("-------------------------");
    puts("Solving direct kinematics");
    puts("-------------------------");

    ros::Rate r(20);
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return (0);
}
