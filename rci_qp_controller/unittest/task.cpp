#include "rci_qp_controller/math/util.hpp"
#include "rci_qp_controller/robot/robot_wrapper.hpp"
#include "rci_qp_controller/tasks/task_se3_equality.hpp"
#include "rci_qp_controller/tasks/task_joint_posture.hpp"

#include "rci_qp_controller/trajectory/trajectory_euclidian.hpp"
#include "rci_qp_controller/trajectory/trajectory_se3.hpp"

#include "rci_qp_controller/math/constraint_bound.hpp"
#include "rci_qp_controller/math/constraint_equality.hpp"
#include "rci_qp_controller/math/constraint_inequality.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <Eigen/QR>    

#include <ros/ros.h>

using namespace qp_controller::math;
using namespace qp_controller::robot;
using namespace qp_controller::tasks;
using namespace qp_controller::trajectory;

using namespace Eigen;
using namespace pinocchio;
using namespace std;

int main (int argc, char** argv)
{
    VectorXd a;

    ros::init(argc, argv, "math_unittest");

    const string franka_model_path = "/home/chan/niryo_ws/src/niryo_robot_description/";
    vector<string> package_dirs;
    package_dirs.push_back(franka_model_path);
    string urdfFileName = package_dirs[0] + "/urdf/one/niryo_grip.urdf";
    RobotWrapper robot(urdfFileName, package_dirs, false);

    const Model& model = robot.model();
    Data data(robot.model());

    Vector q(7), v(7), q_ref(7);
    q.setZero();
    q_ref << 0, -0.785, 0, -2.356, 0, 1.57, 0.78;
    v.setZero();

    int max_it = 100000;
    int na =  robot.nv();
    TaskJointPosture task("task-posture", robot);
    VectorXd Kp = 400. * VectorXd::Ones(na);
    VectorXd Kd = 40. * VectorXd::Ones(na);
    task.Kp(Kp);
    task.Kd(Kd);
    
    TrajectoryEuclidianCubic *traj = new TrajectoryEuclidianCubic("traj_joint");
    TrajectorySample sample;

    double t = 0.0;
    const double dt = 0.01;
    MatrixXd Jpinv(na, na);
    double eps = 1e-5;

    while (t < 2.0){
        if (t ==0){
            traj->setInitSample(q);
            traj->setDuration(1.0);
            traj->setStartTime(0.0);
            traj->setGoalSample(q_ref);
        }
        robot.computeAllTerms(data, q, v);
        traj->setCurrentTime(t);

        sample = traj->computeNext();
        task.setReference(sample);
        auto &constraint = task.compute(t, q, v, data);

        Jpinv = constraint.matrix().completeOrthogonalDecomposition().pseudoInverse();
        ConstRefVector dv = Jpinv * constraint.vector();
        v += dt * dv;
        q = pinocchio::integrate(robot.model(), q, dt * v);
        t += dt;

    }
    cout << "Desired q" << q_ref.transpose() << endl;
    cout << "Final q" << q.transpose() << endl;

    TaskSE3Equality task_se3("task-se3", robot, "fr3_link7");
    task_se3.Kp(Kp);
    task_se3.Kd(Kd);

    pinocchio::SE3 M_ref = robot.framePosition(data, model.getFrameId("fr3_link7"));
    M_ref.translation()(2) -= 0.1;

    TrajectorySE3Cubic *traj_se3 = new TrajectorySE3Cubic("traj_SE3");
    t = 0;
    v.setZero();

    while (t < 1.1){
        if (t ==0){
            traj_se3->setInitSample(robot.framePosition(data, model.getFrameId("fr3_link7")));
            traj_se3->setDuration(1.0);
            traj_se3->setStartTime(0.0);
            traj_se3->setGoalSample(M_ref);
        }
        robot.computeAllTerms(data, q, v);
        traj_se3->setCurrentTime(t);

        sample = traj_se3->computeNext();
        task_se3.setReference(sample);
        auto &constraint_se3 = task_se3.compute(t, q, v, data);
        Jpinv = constraint_se3.matrix().completeOrthogonalDecomposition().pseudoInverse();

        ConstRefVector dv = Jpinv * constraint_se3.vector();
        v += dt * dv;
        q = pinocchio::integrate(robot.model(), q, dt * v);
        t += dt;
    }
   
    cout << "Desired SE3" << M_ref << endl;
    cout << "Final SE3" << robot.position(data, model.getJointId("fr3_joint7")) << endl;
    cout << "q " << q.transpose() << endl;

    return 0;
}
