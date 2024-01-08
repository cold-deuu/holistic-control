#include <Eigen/QR>    
#include <Eigen/Core>
#include <string>
#include <vector>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include "eiquadprog/eiquadprog.hpp"

namespace holistic_controller{
    namespace qp_solver{
        class Holistic_qp{
            public:
                Holistic_qp();

                Eigen::VectorXd m_q,m_err_6d,m_Jm,m_x;
                int m_n;
                Eigen::MatrixXd m_J,m_qlim;
                Eigen::MatrixXd m_Q,m_Aeq,m_Aineq;
                Eigen::VectorXd m_C,m_Beq,m_Bineq;
                double m_p, m_d;

                Eigen::VectorXi m_activeSet;
                size_t m_activeSetSize;
                void initialize();
                void qp_setting(Eigen::VectorXd &error_6d, int &control_num,Eigen::MatrixXd &jacob, Eigen::VectorXd &jacobm, Eigen::MatrixXd &qlimit,Eigen::VectorXd &current_q);
                void gain_setting(double &position_gain, double &velocity_gain);
                void set_Q();
                void set_C();
                void set_eq();
                void set_ineq();
                Eigen::VectorXd solve();
        };
    }
}