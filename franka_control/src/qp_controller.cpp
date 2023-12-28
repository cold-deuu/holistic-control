#include "franka_control/robot.hpp"
#include "franka_control/qp_controller.hpp"

using namespace qp_controller::robot;
using namespace Eigen;
using namespace pinocchio;
using namespace std;

Eigen::MatrixXd J1,J2;
Eigen::VectorXd q1,q2;
Eigen::VectorXd J_m(6);
//q1,J1 과거
//q2,J2 현재

int main(int argc, char** argv)
{
    //Initial Setting
    ros::init(argc,argv,"holistic_panda");
    ros::NodeHandle nh;
    ros::Rate r(10);

    // //pub_ and Sub
    ros::Publisher ur_joint_1 = nh.advertise<std_msgs::Float64>("/shoulder_pan_joint_controller/command",5);
    ros::Publisher ur_joint_2 = nh.advertise<std_msgs::Float64>("/shoulder_lift_joint_controller/command",5);
    ros::Publisher ur_joint_3 = nh.advertise<std_msgs::Float64>("/elbow_joint_controller/command",5);
    ros::Publisher ur_joint_4 = nh.advertise<std_msgs::Float64>("/wrist_1_joint_controller/command",5);
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
    q_.setZero();
    J1.setZero();
    J2.setZero();
    J1.resize(6,6);
    J2.resize(6,6);
    q1.setZero();
    q2.setZero();
    q1.resize(6);
    q2.resize(6);
    r.sleep();

    //joint limits
    qlim_ << -6.28, -6.28, -3.14, -6.28, -6.28, -6.28,
             6.28, 6.28, 3.14, 6.28, 6.28, 6.28;
    qlim_ *= -1;

    //set goal
    pinocchio::SE3 wTep;
    wTep.rotation().setIdentity();
    wTep.translation() << 0.3, 0.4, 0.6;


    //Pose initialize
    joint_publish(q2);

    int k = 1;

    //control
    while (ros::ok()){
        robot.computeAllTerms(data,q_,v_);
        robot.jacobianWorld(data, model.getJointId("wrist_3_joint"), J_); 
        J1 = J2;
        J2 = J_;
        q1 = q2;
        q2 = q_;

        //manipulator jacobian
        double m1 = manipulability(J1);
        double m2 = manipulability(J2);
        Eigen::VectorXd Jm = jacobm(m1,m2,q1,q2);
        if (k<3){
            Jm.setZero();     
        }

        //iteration
        qd_ = step_robot(robot,data,model,wTep,Jm);
        joint_publish(qd_);
        
        ROS_INFO_STREAM((q2-q1).transpose());
        if(et_<0.02){
            ROS_INFO_STREAM("Success!");
            break;
        }
        else if((q2-q1).norm()<0.02 and k>10){
            ROS_INFO_STREAM("FAIL");
            break;
        }
        ROS_WARN_STREAM(Jm);
        k += 1;
        ros::spinOnce();
        r.sleep();

    }

    // ROS_INFO_STREAM("SUCCESS!");
    r.sleep();
    return 0;
}

Eigen::VectorXd & step_robot(const RobotWrapper robot,Data data,Model model,pinocchio::SE3 Tep,Eigen::VectorXd jacobm)
{
    //get Q
    wTe_ = robot.position(data,model.getJointId("wrist_3_joint"));
    eTep_ = wTe_.inverse() * Tep;
    et_ = abs(eTep_.translation()(0)) + abs(eTep_.translation()(1)) + abs(eTep_.translation()(2));
    Q_.setIdentity();
    Q_.topLeftCorner(6,6) = Q_.topLeftCorner(6,6) * 0.01;
    Q_.bottomRightCorner(6,6) = Q_.bottomRightCorner(6,6) /et_;

    //get desired v
    k_gain = k_gain.setIdentity() * 1.5;
    err_.head(3) = eTep_.translation();
    err_.tail(3) = pinocchio::log3(eTep_.rotation()); 
    v_star = k_gain * err_;
    v_star.tail(3) = v_star.tail(3) *1.3;

    //get C(jacobian manipulator)
    Eigen::VectorXd C(12);
    C.head(6) = jacobm;
    C.tail(6).setZero();

    //Equality
    Eigen::MatrixXd Aeq(12,6);
    Eigen::VectorXd Beq(6);
    Aeq.topLeftCorner(6,6) = J_;
    Aeq.topRightCorner(6,6).setIdentity();
    Beq = v_star;

    //Inequality
    Eigen::MatrixXd Aineq(12,6);
    Eigen::VectorXd Bineq(6);

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

    //rest settings for qp
    Eigen::VectorXd x(12);
    Eigen::VectorXi activeSet(0);
    size_t activeSetSize;
    Eigen::VectorXd solution(12);
    solution.head(6) <<q_;
    solution.tail(6).setZero();

    //qp_solve
    double val = 0.0;
    double out = eiquadprog::solvers::solve_quadprog(Q_,C,Aeq,Beq,Aineq,Bineq,x,activeSet,activeSetSize);
    q_d = x.head(6);

    //get desired q_d
    if (et_ > 0.5)
        q_d *= 0.7 / et_;
    else
        q_d *= 1.4;
    
    cout<<"q_d :"<<q_d.transpose()<<endl;
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
        }
        if(msg->name[i] == "wrist_3_joint"){
            q_(5) = msg->position[5];
            v_(5) = msg->velocity[5];
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
    return (Jacob*Jacob.transpose()).determinant();
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
