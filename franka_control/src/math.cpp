#include <cmath>
#include <Eigen/QR>
#include <Eigen/Dense>
#include "franka_control/math.hpp"

using namespace Eigen;
using namespace pinocchio;

namespace holistic_controller{
    namespace math{
        Eigen::MatrixXd pseudoinv(Eigen::MatrixXd Jacob){
            Eigen::MatrixXd eye(6,6);
            eye.setIdentity();
            // return Jacob.transpose() *(Jacob*Jacob.transpose()+0.001*eye).inverse();
            return Jacob.transpose() * (Jacob * Jacob.transpose()).inverse(); //no damp
        }

        double manipulability(Eigen::MatrixXd Jacob)
        {
            return sqrt((Jacob*Jacob.transpose()).determinant());
        }

        Eigen::VectorXd jacobm(double m1, double m2, Eigen::VectorXd q1, Eigen::VectorXd q2)
        {
            Eigen::VectorXd Jm(6);
            for(int i=0; i<6; i++){
                if((q2(i)-q1(i)==0))
                {
                    Jm(i) = 0;
                }
                else{
                    Jm(i) = (m2-m1)/(q2(i) - q1(i));
                }
                
            }
            return Jm;
        }

        Eigen::VectorXd get_error_6d(pinocchio::SE3 &oMi, pinocchio::SE3 &goal_se3){
            pinocchio::SE3 dMi = oMi.inverse() * goal_se3;
            Eigen::Vector3d delphi = GetPhi(oMi.rotation(), goal_se3.rotation());

            Eigen::VectorXd err(6);
            err.head(3) = goal_se3.translation() - oMi.translation();
            err.tail(3) = delphi;

            return err;
        }

        Matrix3d AngleAngle_to_Rot(Vector3d axis, double angle) {
            Matrix3d Rot;
            double kx, ky, kz, theta, vt;
            kx = axis(0);
            ky = axis(1);
            kz = axis(2);
            theta = angle;
            vt = 1.0 - cos(theta);


            Rot(0, 0) = kx * kx*vt + cos(theta);
            Rot(0, 1) = kx * ky*vt - kz * sin(theta);
            Rot(0, 2) = kx * kz*vt + ky * sin(theta);
            Rot(1, 0) = kx * ky*vt + kz * sin(theta);
            Rot(1, 1) = ky * ky*vt + cos(theta);
            Rot(1, 2) = ky * kz*vt - kx * sin(theta);
            Rot(2, 0) = kx * kz*vt - ky * sin(theta);
            Rot(2, 1) = ky * kz*vt + kx * sin(theta);
            Rot(2, 2) = kz * kz*vt + cos(theta);

            return Rot;
        }

        Vector3d GetPhi(Matrix3d Rot, Matrix3d Rotd)
        {
        Vector3d phi;
        Vector3d s[3], v[3], w[3];
        for (int i = 0; i < 3; i++) {
        v[i] = Rot.block(0, i, 3, 1);
        w[i] = Rotd.block(0, i, 3, 1);
        s[i] = v[i].cross(w[i]);
        }
        phi = s[0] + s[1] + s[2];
        phi = -0.5* phi;
        return phi;
        }
    }
}

