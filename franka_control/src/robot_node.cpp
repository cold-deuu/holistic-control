#include "franka_control/robot.hpp"
#include "franka_control/Trajectory.hpp"
#include "franka_control/math.hpp"
#include "franka_control/robot_node.hpp"
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


int main(int argc, char** argv)
{
    //Initial Setting
    ros::init(argc,argv,"holistic_panda");
    ros::NodeHandle nh;
    ros::Rate r(1000);

    // //pub_ and Sub
    ros::Publisher ur_joint_1 = nh.advertise<std_msgs::Float64>("/shoulder_pan_joint_controller/command",5);
    ros::Publisher ur_joint_2 = nh.advertise<std_msgs::Float64>("/shoulder_lift_joint_controller/command",5);
    ros::Publisher ur_joint_3 = nh.advertise<std_msgs::Float64>("/elbow_joint_controller/command",5);
    ros::Publisher ur_joint_4 = nh.advertise <std_msgs::Float64>("/wrist_1_joint_controller/command",5);
    ros::Publisher ur_joint_5 = nh.advertise<std_msgs::Float64>("/wrist_2_joint_controller/command",5);
    ros::Publisher ur_joint_6 = nh.advertise<std_msgs::Float64>("/wrist3_joint_controller/command",5);

    pub_.push_back(ur_joint_1);
    pub_.push_back(ur_joint_2);
    pub_.push_back(ur_joint_3);
    pub_.push_back(ur_joint_4);
    pub_.push_back(ur_joint_5);
    pub_.push_back(ur_joint_6);

    joint_ctrl_.push_back(std_msgs::Float64());
    joint_ctrl_.push_back(std_msgs::Float64());
    joint_ctrl_.push_back(std_msgs::Float64());
    joint_ctrl_.push_back(std_msgs::Float64());
    joint_ctrl_.push_back(std_msgs::Float64());
    joint_ctrl_.push_back(std_msgs::Float64());

    sub_ = nh.subscribe("joint_states",1000 ,&JointStateCallback);
    
    //RobotWrapper
    const string ur_model_path = "/home/chan/holistic_panda/src/universal_robot";
    vector<string> package_dirs;
    package_dirs.push_back(ur_model_path);
    string urdfFileName = package_dirs[0] + "/ur_description/urdf/ur5.urdf";
    RobotWrapper robot(urdfFileName, package_dirs, false);
    const Model &model = robot.model();
    Data data(robot.model());
    nq_ = robot.nq();

    //qp_solver
    Holistic_qp qp_con;

    //Local Variable
    pinocchio::SE3 pose_init_;
    pinocchio::SE3 pose_cubic;
    pinocchio::SE3 pose_current;
    Eigen::VectorXd q_init(6);
    Eigen::MatrixXd J1,J2; //for compute numerical jacobian manipulabiliy
    Eigen::VectorXd q1,q2; //for compute numerical jacobian manipulabiliy
    string ee_id = "wrist_3_joint";
   
    //iteration variable
    Eigen::VectorXd err;

    //initialize
    q_init << 0.0, -1.0, 1.0, 0.0, 0.0, 0.0;
    q_.setZero();
    J1.setZero();
    J2.setZero();
    J1.resize(6,6);
    J2.resize(6,6);
    q1.resize(6);
    q2.resize(6);
    q1.setZero();
    q2.setZero();
    ros::Duration(0.5).sleep();


    //joint limits
    Eigen::MatrixXd qlim(2,nq_);
    qlim << -6.28, -6.28, -3.14, -6.28, -6.28, -6.28,
            6.28, 6.28, 3.14, 6.28, 6.28, 6.28;
    // qlim_ *= -1;

    //set goal
    pinocchio::SE3 wTep;
    wTep.rotation().setIdentity();
    wTep.translation() << 0.3, 0.2, 0.6;

    //Pose initialize
    joint_publish(q_init);
    ros::Rate(0.5).sleep(); //2 seconds wait(to complete initial pose)
    ros::spinOnce();

    //get initial pose
    robot.computeAllTerms(data,q_,v_);
    pose_init_ = robot.position(data,model.getJointId(ee_id));
    ROS_INFO_STREAM("Initialize");

    //iteration settings
    int k = 1;
    double pose_gain = 1000;
    double vel_gain = 150;
    ros::Time stime = ros::Time::now();
    ros::Duration duration = ros::Duration(3.0);

    
    while (ros::Time::now().toSec() <= stime.toSec() + duration.toSec()+1.0)
    {   
        //init iteration
        robot.computeAllTerms(data,q_,v_);
        robot.jacobianWorld(data, model.getJointId(ee_id), J_);

        //manipulator jacobian
        q1 = q2;
        q2 = q_;
        J1 = J2;
        J2 = J_;

        double m1 = holistic_controller::math::manipulability(J1);
        double m2 = holistic_controller::math::manipulability(J2);
        Eigen::VectorXd Jm = holistic_controller::math::jacobm(m1,m2,q1,q2);
        
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
        joint_publish(qd_d);

        k += 1;
        if ((pose_current.translation() - wTep.translation()).norm()<0.02){
            ROS_WARN_STREAM("SUCCES");
            break;
        }

        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO_STREAM(robot.position(data,model.getJointId(ee_id)));

    r.sleep();
    return 0;
}


void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i=0; i<nq_; i++){
        if(msg->name[i] == "shoulder_pan_joint"){
            q_(0) = msg->position[2];
            v_(0) = msg->velocity[2];
        }
        if(msg->name[i] == "shoulder_lift_joint"){
            q_(1) = msg->position[1];
            v_(1) = msg->velocity[1];
        }
        if(msg->name[i] == "elbow_joint"){
            q_(2) = msg->position[0];
            v_(2) = msg->velocity[0];
        }
        if(msg->name[i] == "wrist_1_joint"){
            q_(3) = msg->position[3];
            v_(3) = msg->velocity[3];
        }
        if(msg->name[i] == "wrist_2_joint"){
            q_(4) = msg->position[4];
            v_(4) = msg->velocity[4];
        }
        if(msg->name[i] == "wrist_3_joint"){
            q_(5) = msg->position[5];
            v_(5) = msg-> position[5];
        }
    }
}

    

void joint_publish(Eigen::VectorXd q)
{
    for(int i=0; i<nq_; i++){
        joint_ctrl_[i].data = q(i);
        pub_[i].publish(joint_ctrl_[i]);
    }
}
