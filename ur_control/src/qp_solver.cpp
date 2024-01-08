#include "ur_control/qp_solver.hpp"
#include <ros/ros.h>
using namespace Eigen;

namespace holistic_controller{
    namespace qp_solver{
        
        Holistic_qp::Holistic_qp()
        {
            m_err_6d.resize(6);
        }

        void Holistic_qp::initialize(){
            m_Aeq.setZero();
            m_Beq.setZero();
            m_Q.setZero();
            m_C.setZero();
            m_Aineq.setZero();
            m_Bineq.setZero();
            m_x.setZero();
        }

        void Holistic_qp::qp_setting(Eigen::VectorXd &error_6d, int &control_num,Eigen::MatrixXd &jacob, Eigen::VectorXd &jacobm, Eigen::MatrixXd &qlimit, Eigen::VectorXd &current_q){
            m_err_6d = error_6d;
            m_n = control_num;
            m_J = jacob;
            m_Jm = jacobm;
            m_qlim = qlimit;
            m_q = current_q;
        }

        void Holistic_qp::gain_setting(double &position_gain, double &velocity_gain){
            m_p = position_gain;
            m_d = velocity_gain;
        }

        void Holistic_qp::set_Q(){
            double et = abs(m_err_6d(0)) + abs(m_err_6d(1)) + abs(m_err_6d(2));
            m_Q.resize(m_n+6,m_n+6);
            m_Q.setIdentity();
            m_Q.topLeftCorner(m_n,m_n) = m_Q.topLeftCorner(m_n,m_n)* 0.1;
            m_Q.bottomRightCorner(6,6) = m_Q.bottomRightCorner(6,6)/et;
        }

        void Holistic_qp::set_C(){
            m_C.resize(m_n+6);
            m_C.head(m_n) = m_Jm;
            m_C.tail(6).setZero();
        }

        void Holistic_qp::set_eq(){
            Eigen::MatrixXd k_gain(6,6);
            k_gain.setIdentity();
            k_gain *= m_d;

            Eigen::VectorXd v_star = k_gain * m_err_6d;

            m_Aeq.resize(m_n+6, m_n+6);
            m_Beq.resize(6);
            m_Aeq.setZero();
            m_Beq.setZero();

            m_Aeq.topLeftCorner(6,m_n) = m_J;
            m_Aeq.topRightCorner(6,6).setIdentity();
            m_Beq.head(6) = -v_star;
        }

        void Holistic_qp::set_ineq(){
            double pi = 0.9;
            double ps = 0.1;
            double gain = 1.0;
            m_Aineq.resize(m_n+6,m_n+6);
            m_Bineq.resize(m_n);
            m_Aineq.setZero();
            m_Bineq.setZero();

            for (int i=0; i<m_n; i++)
            {
                if(m_q(i) - m_qlim(0,i)<=m_qlim(1,i) - m_q(i)){
                    m_Bineq(i) = gain * (((m_qlim(0,i) - m_q(i))+ps)/(pi-ps));
                    m_Aineq(i,i) = 1;
                }
                else{
                    m_Bineq(i) = gain * (((m_qlim(1,i) - m_q(i))-ps)/(pi-ps));
                    m_Aineq(i,i) = 1;
                }
                
            }
            m_Aineq *= -1;            
        }

        Eigen::VectorXd Holistic_qp::solve(){
            Holistic_qp::set_Q();
            Holistic_qp::set_C();
            Holistic_qp::set_eq();
            Holistic_qp::set_ineq();
            // ROS_INFO_STREAM(m_Q);
            // ROS_INFO_STREAM(m_C.transpose());
            // ROS_INFO_STREAM(m_Aeq);
            // ROS_INFO_STREAM(m_Beq);
            // ROS_INFO_STREAM(m_Aineq);
            // ROS_INFO_STREAM(m_Bineq);

            // ROS_WARN_STREAM(m_qlim);
            // ROS_WARN_STREAM(m_q);
            m_x.resize(m_n+6);
            double val = 0.0;
            double out = eiquadprog::solvers::solve_quadprog(m_Q,m_C,m_Aeq.transpose(),m_Beq,m_Aineq.transpose(),m_Bineq,m_x,m_activeSet,m_activeSetSize);
            
            return m_x.head(m_n);
        }
    }
}