#include "rci_qp_controller/math/constraint_bound.hpp"
#include "rci_qp_controller/math/constraint_equality.hpp"
#include "rci_qp_controller/math/constraint_inequality.hpp"

#include <ros/ros.h>


using namespace qp_controller::math;
using namespace Eigen;

int main (int argc, char** argv)
{
    ros::init(argc, argv, "math_unittest");

    std::cout << "test_constraint_bounds\n";
    int n = 5;
    VectorXd lb = -1.0 * VectorXd::Ones(n);
    VectorXd ub = VectorXd::Ones(n);
    ConstraintBound bounds("bounds", lb, ub);
    lb *= 2.0;
    bounds.setLowerBound(lb);
    ub *= 2.0;
    bounds.setUpperBound(ub);
    
    std::cout << "upper bound" << bounds.upperBound().transpose() << std::endl;
    std::cout << "lower bound" << bounds.lowerBound().transpose() << std::endl;

    std::cout << "test_constraint_equality\n";
    n = 5;
    int m = 2;
    MatrixXd A = MatrixXd::Ones(m, n);
    VectorXd b = VectorXd::Ones(m);
    ConstraintEquality equality("equality", A, b);

    std::cout << "Matrix" << equality.matrix() << std::endl;
    std::cout << "Vector" << equality.vector().transpose() << std::endl;

    std::cout << "test_constraint_inequality\n";
    ConstraintInequality inequality("inequality", A, lb, ub);

    std::cout << "Matrix" << inequality.matrix() << std::endl;
    std::cout << "upper bound" << inequality.upperBound().transpose() << std::endl;
    std::cout << "lower bound" << inequality.lowerBound().transpose() << std::endl;

    return 0;
    
}