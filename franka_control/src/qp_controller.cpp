#include "franka_control/robot.hpp"
#include "franka_control/qp_controller.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <cmath>
using namespace holistic_controller::robot;
using namespace Eigen;
using namespace pinocchio;
using namespace std;

Eigen::MatrixXd J1,J2;
Eigen::VectorXd q1,q2;
Eigen::VectorXd J_m(6);
Eigen::VectorXd q_d_cubic;
pinocchio::SE3 pose_init_;
pinocchio::SE3 pose_cubic_;
pinocchio::SE3 pose_current_;
pinocchio::SE3 dMi;
Eigen::VectorXd J_inv;
Eigen::VectorXd dv(6);
Eigen::VectorXd dq(6);
Eigen::VectorXd v_d_;
//q1,J1 과거
//q2,J2 현재

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

    //initialize
    Eigen::VectorXd q_init(6);
    q_init << 0.0, -1.0, 1.0, 0.0, 0.0, 0.0;
    q_.setZero();
    J1.setZero();
    J2.setZero();
    J1.resize(6,6);
    J2.resize(6,6);
    q1.setZero();
    q2.setZero();
    q1.resize(6);
    q2.resize(6);
    slack_.setZero();
    ros::Duration(0.5).sleep();

    //joint limits
    qlim_ << -6.28, -6.28, -3.14, -6.28, -6.28, -6.28,
             6.28, 6.28, 3.14, 6.28, 6.28, 6.28;
    // qlim_ *= -1;

    //set goal
    pinocchio::SE3 wTep;
    wTep.rotation().setIdentity();
    wTep.translation() << -0.2, 0.2, 0.6;

    //Pose initialize
    joint_publish(q_init);
    ros::Rate(0.5).sleep();
    ros::spinOnce();
    robot.computeAllTerms(data,q_,v_);
    pose_init_ = robot.position(data,model.getJointId("wrist_3_joint"));
    ROS_INFO_STREAM("Initialize");

    //control
    int k = 1;
    ros::Time stime = ros::Time::now();
    ros::Duration duration = ros::Duration(3.0);
    while (ros::Time::now().toSec() <= stime.toSec() + duration.toSec()+1.0)
    {   
        robot.computeAllTerms(data,q_,v_);
        robot.jacobianWorld(data, model.getJointId("wrist_3_joint"), J_);

        //manipulator jacobian
        q1 = q2;
        q2 = q_;
        J1 = J2;
        J2 = J_;

        double m1 = manipulability(J1);
        double m2 = manipulability(J2);
        Eigen::VectorXd Jm = jacobm(m1,m2,q1,q2);
        if (k<3){
            Jm.setZero();     
        }
        //get trajectory
        pose_cubic_ = se3_cubic(stime,pose_init_,wTep,duration,ros::Time::now());
        
        //iteration
        J_inv = pseudoinv();
        qd_ = step_robot(robot,data,model,pose_cubic_,wTep,Jm);
        pose_current_ = robot.position(data,model.getJointId("wrist_3_joint"));
        Eigen::VectorXd qd_d = pinocchio::integrate(robot.model(),q_,qd_*0.001);
        joint_publish(qd_d);

        k += 1;
        if ((pose_current_.translation() - wTep.translation()).norm()<0.02){
            ROS_WARN_STREAM("SUCCES");
            break;
        }

        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO_STREAM(robot.position(data,model.getJointId("wrist_3_joint")));

    r.sleep();
    return 0;
}

Eigen::VectorXd & step_robot(const RobotWrapper robot,Data &data,Model model,pinocchio::SE3 traj_se3,pinocchio::SE3 Tep,Eigen::VectorXd jacobm)
{
    //initialize
    v_star.resize(6);
    v_star.setZero();
    q_d.setZero();
    err_.setZero();
    
    //get dMi
    wTe_ = robot.position(data,model.getJointId("wrist_3_joint")); 
    Vector3d delphi = GetPhi(wTe_.rotation(), traj_se3.rotation());

    //error
    err_.head(3) = (traj_se3.translation() - wTe_.translation());
    err_.tail(3) = delphi;

    //get Q
    et_ = abs(err_(0)) + abs(err_(1)) + abs(err_(2));
    Q_.setIdentity();
    Q_.topLeftCorner(6,6) = Q_.topLeftCorner(6,6)* 0.1;
    Q_.bottomRightCorner(6,6) = Q_.bottomRightCorner(6,6)/et_;
    
    //get desired v
    k_gain = k_gain.setIdentity() * 100;
    v_star = k_gain * err_;
    // v_star.tail(3) *= 1.3;
    // err_.head(3) = eTep_.translation();
    // err_.tail(3) = pinocchio::log3(eTep_.rotation()); 
    // v_star.head(3) = err_.head(3) * 1000;
    // v_star.tail(3) = err_.tail(3) * 1000;

    //get C(jacobian manipulator)
    Eigen::VectorXd C(12);
    C.head(6) = jacobm;
    C.tail(6).setZero();

    //Equality
    Eigen::MatrixXd Aeq(12,12);
    Eigen::VectorXd Beq(6);
    Aeq.setZero();
    Aeq.topLeftCorner(6,6) = J_;
    Aeq.topRightCorner(6,6).setIdentity();
    Beq.head(6) = -v_star;

    //Inequality
    Eigen::MatrixXd Aineq(12,12);
    Eigen::VectorXd Bineq(12);
    Aineq.setZero();
    Bineq.setZero();

    double pi = 0.9;
    double ps = 0.1;
    double gain = 1.0;

    for (int i=0; i<6; i++)
    {
        if(q_(i) - qlim_(0,i)<=pi){
            Bineq(i) = -gain * (((qlim_(0,i) - q_(i))+ps)/(pi-ps));
            Aineq(i,i) = -1;
        }
        if(qlim_(1,i) - q_(i)<=pi){
            Bineq(i) = gain * (((qlim_(1,i) - q_(i))-ps)/(pi-ps));
            Aineq(i,i) = 1;
        }
    }
    Aineq *= -1;
    // Bineq *= -1;


    //rest settings for qp
    Eigen::VectorXd x(12);
    Eigen::VectorXi activeSet;
    size_t activeSetSize;
    // Eigen::VectorXd solution(12);
    // solution.head(6).setZero();
    // solution.tail(6).setZero();

    // C.setZero();

    //qp_solve
    double val = 0.0;
    double out = eiquadprog::solvers::solve_quadprog(Q_,C,Aeq.transpose(),Beq,Aineq.transpose(),Bineq,x,activeSet,activeSetSize);


    q_d = x.head(6);
    slack_= x.tail(6);
    // get desired q_d
    if (et_ > 0.5)
        q_d *= 0.7 / et_;
    else if(et_<=0.5 and et_>0.02)
        q_d *= 1.4;
    else
        q_d *= 0.0;
    return q_d;
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i=0; i<6; i++){
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
        
        if(msg->name[i]) == "wrist_3_joint"{
            q_(5) = msg->position[5];
            v_(5) = msg-> position[5];
        }
        }
    }
}
    

void joint_publish(Eigen::VectorXd q)
{
    for(int i=0; i<6; i++){
        joint_ctrl_[i].data = q(i);
        pub_[i].publish(joint_ctrl_[i]);
    }
}


double manipulability(Eigen::MatrixXd Jacob)
{
    return sqrt((Jacob*Jacob.transpose()).determinant());
}

Eigen::VectorXd jacobm(double m1, double m2, Eigen::VectorXd q1, Eigen::VectorXd q2)
{
    for(int i=0; i<6; i++){
        if((q2(i)-q1(i)==0))
        {
            J_m(i) = 0;
        }
        else{
            J_m(i) = (m2-m1)/(q2(i) - q1(i));
        }
        
    }
    return J_m;
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
    ROS_INFO_STREAM(se3_cubic.translation().transpose());
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


Eigen::MatrixXd pseudoinv(){
    Eigen::MatrixXd eye(6,6);
    eye.setIdentity();
    // return J_.transpose() *(J_*J_.transpose()+0.001*eye).inverse();
    return J_.transpose() * (J_ * J_.transpose()).inverse(); //no damp
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
