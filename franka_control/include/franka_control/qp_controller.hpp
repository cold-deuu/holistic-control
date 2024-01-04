#include <Eigen/QR>    
#include <Eigen/Core>
#include <ros/ros.h>
#include "eiquadprog/eiquadprog.hpp"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <stdio.h>
#include <tuple>
#include <sys/ioctl.h>
#include <termios.h>
#include "franka_control/robot.hpp"
#include "franka_control/qp_solver.hpp"
#include "franka_control/Trajectory.hpp"


using namespace holistic_controller::robot;
using namespace std;
using namespace pinocchio;
using namespace Eigen;

//varialbes
vector<ros::Publisher> pub_;
// ros::Publisher pub_;
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
Eigen::VectorXd slack_(6);




// ros::Publisher pub_;
ros::Subscriber sub_;

//controller_function
Eigen::VectorXd & step_robot(const RobotWrapper robot,Data &data,Model model,pinocchio::SE3 traj_se3,pinocchio::SE3 Tep,Eigen::VectorXd jacobm);
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void joint_publish(Eigen::VectorXd q);
double manipulability(Eigen::MatrixXd Jacob);
Eigen::Matrix3d AngleAngle_to_Rot(Vector3d axis, double angle);
Eigen::VectorXd jacobm(double m1, double m2, Eigen::VectorXd q1, Eigen::VectorXd q2);
Eigen::VectorXd q_cubic(ros::Time &ros_stime,Eigen::VectorXd &q_init, Eigen::VectorXd &q_dot_init, Eigen::VectorXd &q_goal,ros::Duration &ros_duration, ros::Time &ros_time);
pinocchio::SE3 se3_cubic(ros::Time &ros_stime, pinocchio::SE3 &se3_init, pinocchio::SE3 &se3_goal, ros::Duration &ros_duration, ros::Time ros_time);
Eigen::MatrixXd pseudoinv();
Vector3d GetPhi(Matrix3d Rot, Matrix3d Rotd);
