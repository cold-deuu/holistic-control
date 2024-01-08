#include <Eigen/QR>    
#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <ros/ros.h>

using namespace Eigen;

namespace holistic_controller{
    namespace trajectory{
        Eigen::VectorXd q_cubic(ros::Time &ros_stime,Eigen::VectorXd &q_init, Eigen::VectorXd &q_dot_init, Eigen::VectorXd &q_goal,ros::Duration &ros_duration, ros::Time &ros_time);
        pinocchio::SE3 se3_cubic(ros::Time &ros_stime, pinocchio::SE3 &se3_init, pinocchio::SE3 &se3_goal, ros::Duration &ros_duration, ros::Time ros_time);
    }
}
