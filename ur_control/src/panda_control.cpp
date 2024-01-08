#include "ur_control/robot.hpp"
#include "ur_control/Trajectory.hpp"
#include "ur_control/math.hpp"
#include "ur_control/robot_node.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <cmath>
using namespace holistic_controller::robot;
using namespace holistic_controller::math;
using namespace holistic_controller::trajectory;
using namespace holistic_controller::qp_solver;
using namespace Eigen;
using namespace pinocchio;
using namespace std;

int nq_;
ros::Publisher panda_pub_;


int main(int argc, char** argv)
{
    //Initial Setting
    ros::init(argc,argv,"holistic_panda_con");
    ros::NodeHandle nh;
    ros::Rate r(1000);

    // //pub_ and Sub
    panda_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/robot_arm_controller/command",5);
    sub_ = nh.subscribe("joint_states",1000 ,&JointStateCallback);
    
    //RobotWrapper
    const string ur_model_path = "/home/chan/holistic_panda/src";
    vector<string> package_dirs;
    package_dirs.push_back(ur_model_path);
    string urdfFileName = package_dirs[0] + "/robot_description/robots/franka_emika_panda_robot.urdf";
    RobotWrapper robot(urdfFileName, package_dirs, false);
    const Model &model = robot.model();
    Data data(robot.model());
    nq_ = robot.nq();


    //qp_solver
    Holistic_qp qp_con;

    //Local Variable
    pinocchio::SE3 a_pose;
    pinocchio::SE3 wTep;
    pinocchio::SE3 pose_init_;
    pinocchio::SE3 pose_cubic;
    pinocchio::SE3 pose_current;
    Eigen::VectorXd q_init(nq_);
    Eigen::MatrixXd J1,J2; //for compute numerical jacobian manipulabiliy
    Eigen::VectorXd q1,q2; //for compute numerical jacobian manipulabiliy
    string ee_id = "panda_joint7";

    //iteration variable
    Eigen::VectorXd err;

    //initialize
    q_init << 0.1, 0.4, 0.4, 0.4, 0.4, 0.4,0.4;
    q_.resize(nq_);
    q_.setZero();
    v_.resize(nq_);
    v_.setZero();
    J1.setZero();
    J2.setZero();
    J1.resize(6,nq_);
    J2.resize(6,nq_);
    q1.resize(nq_);
    q2.resize(nq_);
    q1.setZero();
    q2.setZero();

    J_.resize(6,nq_);
    ros::Duration(0.5).sleep();


    //joint limits
    Eigen::MatrixXd qlim(2,nq_);
    qlim << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8793,
            2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8793;
    // qlim *= -1;
    
    a_pose = robot.position(data,model.getJointId(ee_id));
    //Pose initialize
    joint_publish(q_init);
    ros::Rate(0.5).sleep(); //2 seconds wait(to complete initial pose)
    ros::spinOnce();

    //get initial pose
    robot.computeAllTerms(data,q_,v_);
    pose_init_ = robot.position(data,model.getJointId(ee_id));
    ROS_INFO_STREAM("Initialize");
    ROS_INFO_STREAM(pose_init_);
    
    //set goal
    wTep.rotation().setIdentity();
    wTep.translation() << 0.3, 0.4, 0.8;
    
    ROS_WARN_STREAM(q_);
    //iteration settings
    int k = 1;
    double pose_gain = 100;
    double vel_gain = 130;
    ros::Duration(0.5).sleep();

    ros::Time stime = ros::Time::now();
    ros::Duration duration = ros::Duration(3.0);
    
    while (ros::Time::now().toSec() <= stime.toSec() + duration.toSec()+1.0)
    {   
        //init iteration
        robot.jacobianWorld(data, model.getJointId(ee_id), J_);
        robot.computeAllTerms(data,q_,v_);

        //manipulator jacobian
        q1 = q2;
        q2 = q_;
        J1 = J2;
        J2 = J_;

        double m1 = holistic_controller::math::manipulability(J1);
        double m2 = holistic_controller::math::manipulability(J2);
        Eigen::VectorXd Jm = holistic_controller::math::jacobm(m1,m2,q1,q2);

        // ROS_WARN_STREAM(q1);
        // ROS_WARN_STREAM(q2);        
        // ROS_WARN_STREAM(Jm.transpose());
        // ROS_INFO_STREAM(q_.transpose());
        if (k<3){
            Jm.setZero();     
        }


        //get trajectory
        pose_cubic = holistic_controller::trajectory::se3_cubic(stime,pose_init_,wTep,duration,ros::Time::now());
        pose_current = robot.position(data,model.getJointId(ee_id));
        //qp_solve
        err = holistic_controller::math::get_error_6d(pose_current,pose_cubic);
        
        qp_con.initialize();
        qp_con.qp_setting(err,nq_,J_,Jm,qlim,q_);
        qp_con.gain_setting(pose_gain,vel_gain);
        qd_ = qp_con.solve();

        
        //iteration
        Eigen::VectorXd qd_d = pinocchio::integrate(robot.model(),q_,qd_*0.001);
        for(int i=0; i<nq_; i++)
        {   
            if(std::isnan(qd_d(i))){
                qd_d(i) = q_(i);
                ROS_WARN_STREAM("WARN");
            }
                
        }

        ROS_INFO_STREAM(qd_d);
        joint_publish(qd_d);

        
        ros::spinOnce();
        r.sleep();
        k += 1;
        if ((pose_current.translation() - wTep.translation()).norm()<0.02){
            ROS_WARN_STREAM("SUCCES");
            ROS_WARN_STREAM(ros::Time::now().toSec() - stime.toSec());
            break;
        }

    }

    ROS_INFO_STREAM(robot.position(data,model.getJointId(ee_id)));
    return 0;
}



void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    for(int i=0;i<nq_;i++){
        q_(i) = msg->position[i+2];
        v_(i) = msg->velocity[i+2];
    }
}

    

// void joint_publish(Eigen::VectorXd q)
// {
//     for(int i=0; i<nq_; i++){
//         joint_ctrl_[i].data = q(i);
//         pub_[i].publish(joint_ctrl_[i]);
//     }
// }

void joint_publish(Eigen::VectorXd q){
    std_msgs::Float64MultiArray msg;
    for(int i=0;i<nq_;i++){
        msg.data.push_back(q(i));
    }
    panda_pub_.publish(msg);
    
}