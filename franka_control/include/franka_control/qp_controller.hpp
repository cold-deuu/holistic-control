#include <Eigen/QR>    
#include <Eigen/Core>
#include <ros/ros.h>
#include <eiquadprog/eiquadprog.hpp>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <stdio.h>
#include <tuple>
#include <ros/ros.h>
#include <sys/ioctl.h>
#include <termios.h>

using namespace qp_controller::robot;
using namespace std;
using namespace pinocchio;

//varialbes
vector<ros::Publisher> pub_;
pinocchio::SE3 wTe_;
pinocchio::SE3 eTep_;
Eigen::MatrixXd Q_(12,12);
double et_;                 //error of translation (Spatial error)
Eigen::VectorXd err_(6);
Eigen::MatrixXd k_gain(6,6);
Eigen::VectorXd v_star;
Eigen::VectorXd v_(6);
Eigen::VectorXd q_(6);
Eigen::VectorXd q_goal_(6);
ros::Publisher a,b,c,d,e,f;
vector<std_msgs::Float64> joint_ctrl_;
Eigen::VectorXd qd_(6);
Eigen::MatrixXd J_(6,6);
Eigen::VectorXd qlim_(2,6);
Eigen::VectorXd q_d(6);



// ros::Publisher pub_;
ros::Subscriber sub_;

//controller_function
Eigen::VectorXd & step_robot(const RobotWrapper robot,Data data,Model model,pinocchio::SE3 Tep,Eigen::VectorXd jacobm);
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void joint_publish(Eigen::VectorXd q);
double manipulability(Eigen::MatrixXd Jacob);
Eigen::VectorXd jacobm(double m1, double m2, Eigen::VectorXd q1, Eigen::VectorXd q2);
