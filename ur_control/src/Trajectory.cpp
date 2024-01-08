#include "ur_control/Trajectory.hpp"
#include "ur_control/math.hpp"

using namespace Eigen;
using namespace pinocchio;
using namespace holistic_controller::math;

namespace holistic_controller{
    namespace trajectory{
        Eigen::VectorXd q_cubic(ros::Time &ros_stime,Eigen::VectorXd &q_init, Eigen::VectorXd &q_dot_init, Eigen::VectorXd &q_goal,ros::Duration &ros_duration, ros::Time &ros_time){
            
            //Property
            Eigen::VectorXd q_cubic = q_init;
            double a0,a1,a2,a3;
            double stime = ros_stime.toSec();
            double time = ros_time.toSec();
            double duration = ros_duration.toSec();

            if(time<stime){
                return q_init;
            }
            else if(time> stime + duration){
                return q_goal;
            }
            else{
                for(int i=0;i<q_cubic.size();i++){
                    a0 = q_init(0);
                    a1 = q_dot_init(0);
                    a2 = 3/pow(duration,2) *(q_goal(i)-q_init(i));
                    a3 = -1.0 *2.0/pow(duration,3) *(q_goal(i)-q_init(i));
                    q_cubic(i) = a0 + a1 * (time - stime) + a2*pow(time-stime,2) + a3 * pow(time-stime,3);
                }
                return q_cubic;
            }
                
        }

        pinocchio::SE3 se3_cubic(ros::Time &ros_stime, pinocchio::SE3 &se3_init, pinocchio::SE3 &se3_goal, ros::Duration &ros_duration, ros::Time ros_time){
            
            //Property
            pinocchio::SE3 se3_cubic;
            se3_cubic = se3_init;
            double a0,a1,a2,a3;

            double stime = ros_stime.toSec();
            double time = ros_time.toSec();
            double duration = ros_duration.toSec();
            double current_angle, init_angle, goal_angle;

            Eigen::Vector3d se3_init_trans = se3_init.translation();
            Eigen::Vector3d se3_goal_trans = se3_goal.translation();
            Eigen::Matrix3d goal_rot = se3_goal.rotation();
            Eigen::Matrix3d init_rot = se3_init.rotation();
            Eigen::Matrix3d temp_rot;

            //Translation trajectory
            if(time<stime){
                se3_cubic.translation() = se3_init_trans;
            }
            else if(time> stime + duration){
                se3_cubic.translation() = se3_goal_trans;
            }
            else{
                for(int i=0;i<3;i++){
                    a0 = se3_init_trans(i);
                    a1 = 0.0; //m_init.vel(i);
                    a2 = 3.0 / pow(duration, 2) * (se3_goal_trans(i) - se3_init_trans(i));
                    a3 = -1.0 * 2.0 / pow(duration, 3) * (se3_goal_trans(i) - se3_init_trans(i));
                    se3_cubic.translation()(i) = a0 + a1 * (time - stime) + a2*pow(time-stime,2) + a3 * pow(time-stime,3);
                }
            }
            //Angle trajectory
            Eigen::Matrix3d rot_diff = init_rot.inverse() * goal_rot;
            Eigen::AngleAxisd A(rot_diff);
            Eigen::Vector3d rot_diff_3d = pinocchio::log3(rot_diff);
            goal_angle = A.angle();
            init_angle = 0.0;

            if(time<stime){
                current_angle = init_angle;
            }
            else if(time> stime + duration){
                current_angle = goal_angle;
            }
            else{
                a0 = init_angle;
                a1 = 0.0;
                a2 = 3/pow(duration,2) *(goal_angle-init_angle);
                a3 = -1.0 *2.0/pow(duration,3) *(goal_angle-init_angle);
                current_angle = a0 + a1 * (time - stime) + a2*pow(time-stime,2) + a3 * pow(time-stime,3);
            }

            se3_cubic.rotation() = se3_init.rotation() * AngleAngle_to_Rot(A.axis(),current_angle);
            // se3_cubic.rotation() = se3_init.rotation() * pinocchio::exp3(rot_diff_3d * tau);
            return se3_cubic;
        }
    }
}
