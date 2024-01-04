#include "rci_qp_controller/math/util.hpp"
#include "rci_qp_controller/robot/robot_wrapper.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <ros/ros.h>

using namespace qp_controller::math;
using namespace qp_controller::robot;
using namespace Eigen;
using namespace pinocchio;
using namespace std;

int main (int argc, char** argv)
{
    ros::init(argc, argv, "math_unittest");

    const string franka_model_path = "/home/home/tutorial_ws/src/franka_ros/franka_description/";
    vector<string> package_dirs;
    package_dirs.push_back(franka_model_path);
    string urdfFileName = package_dirs[0] + "/robots/fr3/fr3.urdf";
    RobotWrapper robot(urdfFileName, package_dirs, false);

    const Model& model = robot.model();
    ROS_WARN_STREAM(model);

    Vector q(7), v(7);
    q.setZero();
    v.setZero();

    Data data(robot.model());
    robot.computeAllTerms(data, q, v);
    MatrixXd M = robot.mass(data);

    ROS_WARN_STREAM(M);

    return 0;
}