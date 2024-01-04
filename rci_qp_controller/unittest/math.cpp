#include "rci_qp_controller/math/util.hpp"

#include <ros/ros.h>


using namespace qp_controller::math;
int main (int argc, char** argv)
{
    ros::init(argc, argv, "math_unittest");
    std::cout << "test_pseudoinverse\n";

    const unsigned int m = 3;
    const unsigned int n = 5;

    Matrix A = Matrix::Random(m, n);
    Matrix Apinv = Matrix::Zero(n, m);
    pseudoInverse(A, Apinv, 1e-5);

    std::cout << A*Apinv << std::endl;
	return 0;
}