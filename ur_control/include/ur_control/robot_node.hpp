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
#include <string>
#include <vector>
#include "ur_control/robot.hpp"
#include "ur_control/Trajectory.hpp"
#include "ur_control/math.hpp"
#include "ur_control/qp_solver.hpp"


using namespace holistic_controller::robot;
using namespace std;
using namespace pinocchio;
using namespace Eigen;

//varialbes
vector<ros::Publisher> pub_;
pinocchio::SE3 wTe_;
pinocchio::SE3 eTep_;
Eigen::VectorXd v_;
Eigen::VectorXd q_;
vector<std_msgs::Float64> joint_ctrl_;
Eigen::VectorXd qd_;
Eigen::MatrixXd J_;


// ros::Publisher pub_;
ros::Subscriber sub_;

//controller_function
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void joint_publish(Eigen::VectorXd q);
