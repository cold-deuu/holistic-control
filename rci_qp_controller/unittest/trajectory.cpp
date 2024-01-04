#include "rci_qp_controller/math/util.hpp"
#include "rci_qp_controller/trajectory/trajectory_euclidian.hpp"
#include "rci_qp_controller/trajectory/trajectory_se3.hpp"

#include <ros/ros.h>

using namespace qp_controller::math;
using namespace qp_controller::trajectory;
using namespace Eigen;
using namespace pinocchio;

int main (int argc, char** argv)
{
    ros::init(argc, argv, "math_unittest");

    std::cout << "test se3 trajectory \n";

    SE3 M_ref = SE3::Identity();
    VectorXd M_vec(12);
    SE3ToVector(M_ref, M_vec);
    VectorXd zero = VectorXd::Zero(6);
    TrajectoryBase *traj = new TrajectorySE3Constant("traj_se3", M_ref);


    TrajectorySample sample(12, 6);
    traj->getLastSample(sample);

    std::cout << "first sample" << M_ref << std::endl;
    std::cout << "last sample" << sample.pos.transpose() << std::endl;
    
    std::cout << "test joint trajectory \n";
    const unsigned int n = 5;
    VectorXd q_ref = VectorXd::Ones(n);
    zero = VectorXd::Zero(n);
    TrajectoryBase *j_traj = new TrajectoryEuclidianConstant("traj_eucl", q_ref);
    j_traj->getLastSample(sample);
    std::cout << "first sample" << q_ref.transpose() << std::endl;
    std::cout << "last sample" << sample.pos.transpose() << std::endl;
    

    return 0;
    
}