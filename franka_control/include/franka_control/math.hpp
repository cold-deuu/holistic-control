#include <Eigen/QR>    
#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>

using namespace Eigen;


namespace holistic_controller{
    namespace math{
        Eigen::MatrixXd pseudoinv();
        double manipulability(Eigen::MatrixXd Jacob);
        Eigen::VectorXd jacobm(double m1, double m2, Eigen::VectorXd q1, Eigen::VectorXd q2);
        Eigen::VectorXd get_error_6d(pinocchio::SE3 &oMi, pinocchio::SE3 &goal_se3);
        Matrix3d AngleAngle_to_Rot(Vector3d axis, double angle);
        Vector3d GetPhi(Matrix3d Rot, Matrix3d Rotd);
    }
}

