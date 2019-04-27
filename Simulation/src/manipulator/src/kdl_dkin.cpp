#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <math.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#define MAX_LINEAR_VEL 0.1
#define MAX_ROTATION_VEL 0.1
#define PORT "/dev/ttyACM0"
#define BASE_VEL 0.1

int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0; // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}



int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

char *portname = PORT;
int fd;
int wlen;


class KDLSolver {
public:
    KDLSolver();
    void newStateCallback(const sensor_msgs::JointState &msg);
    void newVelCallback(const geometry_msgs::Twist &msg);
    void newVel(const geometry_msgs::Twist &msg);
    void updatePosition();
    void resetPosition();
    void processKeyboardHit(int character);
    ros::Time time;
    ros::Time dt;
    ros::Time timeout;
    KDL::JntArray jointvelocities;
    static const unsigned int JOINTS_AMOUNT = 6;
    FILE *serialPort;
    int mode = 0;

private:
    geometry_msgs::PoseStamped solveDirectKinematics(const sensor_msgs::JointState &msg);
    KDL::JntArray solveIndirectKinematics(const geometry_msgs::Twist &msg);

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

    // Sub geometry_msgs::Twist
    twist_sub_ = nh_.subscribe("/twist",1,&KDLSolver::newVelCallback, this);


    twist_one_sub_ = nh_.subscribe("/twist_one",1,&KDLSolver::newVel, this);

    // Pub geometry_msgs/PoseStamped
    pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/kdl_dkin", 1);

    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Vector(0, 0, 0.13))));

    KDL::Frame frame = KDL::Frame(KDL::Rotation::RPY(-KDL::PI/2,0.0,0.0),KDL::Vector(0, 0.00175, 0.0705));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    frame = KDL::Frame(KDL::Rotation::RPY(0.0,0.0,1.283),KDL::Vector(0, -0.6, 0.0));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    frame = KDL::Frame(KDL::Rotation::EulerZYX(0.0,-KDL::PI/2,0.0),KDL::Vector(0.408, 0.005, 0.0));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    frame = KDL::Frame(KDL::Rotation::EulerZYX(-KDL::PI/2,0.0,3*KDL::PI/2),KDL::Vector(0.0, 0.0, -0.129));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    frame = KDL::Frame(KDL::Rotation::EulerZYX(0.0,-KDL::PI/2,0.0),KDL::Vector(0.0643, 0.0, 0.0));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    frame = KDL::Frame(KDL::Rotation::EulerZYX(0.0,KDL::PI/2,0.0),KDL::Vector(0.0, 0.0, -0.15));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frame));

    jointvelocities = KDL::JntArray(chain.getNrOfJoints());

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

//    for (unsigned int i = 0; i < JOINTS_AMOUNT; i++) {
//        theta_[i] = msg.position.at(i);
//        if ((theta_[i] < theta_limit_lower_[i]) ||
//            (theta_[i] > theta_limit_upper_[i])) {
//            is_d_kin_pose_reachable_ = false;
//            ROS_WARN("Reached joint %d limit! Not moving further!", (i + 1));
//        }
//    }

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

KDL::JntArray KDLSolver::solveIndirectKinematics(const geometry_msgs::Twist &msg) {

    KDL::Twist temp_twist;
    //Create joint array for velocity results:
    if (!is_d_kin_pose_reachable_)
    {
        ROS_WARN("Joint limit! Can't move any further!");
        for(int i = 0; i < JOINTS_AMOUNT; ++i)
        {
            jointvelocities(i) = 0;
        }
        return jointvelocities;
    }
    temp_twist.vel.x(msg.linear.x);
    temp_twist.vel.y(msg.linear.y);
    temp_twist.vel.z(msg.linear.z);
    temp_twist.rot.x(msg.angular.x);
    temp_twist.rot.y(msg.angular.y);
    temp_twist.rot.z(msg.angular.z);

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
    KDL::ChainIkSolverVel_pinv iksolver(chain);//Inverse velocity solver
    iksolver.CartToJnt(jointpositions,temp_twist, jointvelocities);

//    ChainIkSolverPos_NR iksolve(chain,fksolver,iksolver,100,1e-6);
    // Calculate forward position kinematics
//    fksolver.JntToCart(jointpositions,cartpos);

//    // Fill the position part
//    geometry_msgs::PoseStamped temp_pose;
//
//    temp_pose.pose.position.x = cartpos.p.x();
//    temp_pose.pose.position.y = cartpos.p.y();
//    temp_pose.pose.position.z = cartpos.p.z();
//
//    cartpos.M.GetQuaternion(temp_pose.pose.orientation.x, temp_pose.pose.orientation.y,
//                            temp_pose.pose.orientation.z, temp_pose.pose.orientation.w);

//    // Transformation is happening in base_link
//    temp_pose.header.frame_id = "base_link";

    return jointvelocities;
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

void KDLSolver::newVelCallback(const geometry_msgs::Twist &msg) {
    // Calculate direct kinematics
    solveIndirectKinematics(msg);

    for(int i = 0; i < JOINTS_AMOUNT; ++i)
    {
        ROS_INFO("Velocity %d: %f",i,jointvelocities(i));
    }
    timeout = time.now();
}

void KDLSolver::newVel(const geometry_msgs::Twist &msg) {
    jointvelocities(0) = msg.linear.x;
    jointvelocities(1) = msg.linear.y;
    jointvelocities(2) = msg.linear.z;
    jointvelocities(3) = msg.angular.x;
    jointvelocities(4) = msg.angular.y;
    jointvelocities(5) = msg.angular.z;
    ROS_INFO("NewVel!");
}

void KDLSolver::resetPosition()
{
    for(int i = 0; i < JOINTS_AMOUNT; ++i)
    {
        theta_[i] = 0;
        new_state.position[i] = theta_[i];
    }

    joint_state_pub_.publish(new_state);
}


void KDLSolver::updatePosition()
{
    for(int i = 0; i < JOINTS_AMOUNT; ++i)
    {
        theta_[i]+= jointvelocities(i)*(time.now().toSec()-dt.toSec());
        new_state.position[i] = theta_[i];
    }
    new_state.header.stamp = ros::Time::now();
    dt = time.now();
    joint_state_pub_.publish(new_state);
//    ROS_INFO("Publishing velocities(%f, %f, %f, %f, %f, %f",jointvelocities(0),
//             jointvelocities(1), jointvelocities(2), jointvelocities(3),
//             jointvelocities(4), jointvelocities(5));
    char data[5];
    int temp[4];
    data[0] = 255;
    temp[0] = static_cast<int>(jointvelocities(0)/0.4*63);
    temp[1]= -static_cast<int>(jointvelocities(1)/0.6*40);
    temp[2] = static_cast<int>(jointvelocities(2)/0.25*63);
    temp[3] = 0;
    for(int i = 0; i < 4; ++i)
    {
        if(temp[i] > 63)
        {
            temp[i] = 63;
        }
        else if(temp[i] < -63)
        {
            temp[i] = -63;
        }
    }
    if(temp[2] > 0)
    {
        temp[2] = 63;
    }else if(temp[2] < 0)
    {
        temp[2] = -63;
    }
//    if(temp[1] < 0)
//    {
//        temp[1] = -40;
//    }
    if(temp[1] > 0)
    {
        temp[1] = 55;
    }

    ROS_INFO("Sending velocities via UART-temp-not(%u, %u, %u, %u, %u, %u)",temp[0],
             temp[1], temp[2], temp[3],
             temp[4], temp[5]);
//    ROS_INFO("Sending velocities via UART-temp(%d, %d, %d, %d, %d, %d)",temp[0],
//             temp[1], temp[2], temp[3],
//             temp[4], temp[5]);
    for(int i = 0; i < 4; ++i)
    {
        temp[i] = temp[i]+63;
    }
    ROS_INFO("Sending velocities via UART(%u, %u, %u, %u, %u, %u)",temp[0],
             temp[1], temp[2], temp[3],
             temp[4], temp[5]);
    for(int i = 1; i < 5; ++i)
    {
        data[i] = static_cast<char>(temp[i-1]);
    }
    ROS_INFO("Sending velocities via UART-converted(%d, %d, %d, %d, %d)",data[0],
             data[1], data[2], data[3],
             data[4]);
    wlen = write(fd, data, 5);
    if(wlen != 5)
    {
        ROS_WARN("Port closed! Shutting down.");
    }
    tcdrain(fd);
}

void KDLSolver::processKeyboardHit(int character)
{
    geometry_msgs::Twist tempTwist;
    tempTwist.linear.x = 0;
    tempTwist.linear.y = 0;
    tempTwist.linear.z = 0;
    tempTwist.angular.x = 0;
    tempTwist.angular.y = 0;
    tempTwist.angular.z = 0;
    char c = character;
    ROS_WARN("Pressed key %d", c);
    if(c == 'p')
    {
        resetPosition();
        return;
    }
    else if(character == 'm')
    {
        ROS_WARN("Changing mode to simulation.");
        char data[5];
        data[0] = 255;
        data[1] = 255;
        data[2] = 2;
        data[3] = 2;
        data[4] = 2;
        wlen = write(fd, data, 5);
        mode = 2;
        return;
    }
    else if(character == 'n')
    {
        ROS_WARN("Changing mode to manual.");
        char data[5];
        data[0] = 255;
        data[1] = 255;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        wlen = write(fd, data, 5);
        mode = 0;
        return;
    }

    if(KDLSolver::mode == 0)
    {
        wlen = write(fd, &c, 1);
        tcdrain(fd);
    }
    else if(mode == 2)
    {
        switch(character)
        {
            case 'w':
                tempTwist.linear.x = BASE_VEL;
                break;
            case 's': {
//                tempTwist.linear.x = -BASE_VEL;
                wlen = write(fd, &c, 1);
                tcdrain(fd);
                char d = 'd';
                wlen = write(fd, &d, 1);
                tcdrain(fd);
                break;
            }
            case 'a':
                tempTwist.linear.y = BASE_VEL;
                break;
            case 'd':
                tempTwist.linear.y = -BASE_VEL;
                break;
            case 'r':
                tempTwist.linear.z = BASE_VEL;
                break;
            case 'f':
                tempTwist.linear.z = -BASE_VEL;
                break;
        }
        if(c != 's')
            newVelCallback(tempTwist);

    }
}


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "kdl_dkin");
    KDLSolver kinematics_solver;

    puts("-------------------------");
    puts("Solving direct and indirect kinematics");
    puts("-------------------------");

    kinematics_solver.dt = kinematics_solver.time.now();

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    printf("FD: %d\n", fd);
    wlen = 5;
    set_interface_attribs(fd, B9600);

    ros::Rate r(20);
    while(ros::ok()) {
        int c = getch();
        if(c != -1)
            kinematics_solver.processKeyboardHit(c);
        if(kinematics_solver.mode != 0)
            kinematics_solver.updatePosition();
        if((kinematics_solver.time.now().toSec()-kinematics_solver.timeout.toSec()) > 0.1)
        {
            ROS_INFO("Timeout reached.");
            for(int i = 0; i < kinematics_solver.JOINTS_AMOUNT; ++i)
                kinematics_solver.jointvelocities(i) = 0;
        }
        ros::spinOnce();
        r.sleep();
    }

    return (0);
}
