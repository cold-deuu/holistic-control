#include "rci_qp_controller/trajectory/trajectory_se3.hpp"
#include "rci_qp_controller/math/util.hpp"
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
using namespace qp_controller::math;
using namespace std;

namespace qp_controller
{
  namespace trajectory
  {

    TrajectorySE3Constant::TrajectorySE3Constant(const std::string & name)
      :TrajectoryBase(name)
    {
      m_sample.resize(12, 6);
    }

    TrajectorySE3Constant::TrajectorySE3Constant(const std::string & name,
                                                 const SE3 & M)
      :TrajectoryBase(name)
    {
      m_sample.resize(12, 6);
      SE3ToVector(M, m_sample.pos);
    }

    unsigned int TrajectorySE3Constant::size() const
    {
      return 6;
    }

    void TrajectorySE3Constant::setReference(const pinocchio::SE3 & ref)
    {
      m_sample.resize(12, 6);
      SE3ToVector(ref, m_sample.pos);
    }

    const TrajectorySample & TrajectorySE3Constant::operator()(double )
    {
      return m_sample;
    }

    const TrajectorySample & TrajectorySE3Constant::computeNext()
    {
      return m_sample;
    }

    void TrajectorySE3Constant::getLastSample(TrajectorySample & sample) const
    {
      sample = m_sample;
    }

    bool TrajectorySE3Constant::has_trajectory_ended() const
    {
      return true;
    }
    const std::vector<Eigen::VectorXd> & TrajectorySE3Constant::getWholeTrajectory(){
      traj_.clear();
      traj_.push_back(m_sample.pos);
      return traj_;
    }


    TrajectorySE3Cubic::TrajectorySE3Cubic(const std::string & name)
      :TrajectoryBase(name)
    {}

    TrajectorySE3Cubic::TrajectorySE3Cubic(const std::string & name, const SE3 & init_M, const SE3 & goal_M, const double & duration, const double & stime) 
    : TrajectoryBase(name)
    {
      setGoalSample(goal_M);
      setInitSample(init_M);
      setDuration(duration);
      setStartTime(stime);
    }
    unsigned int TrajectorySE3Cubic::size() const
    {
      return 6;
    }

    const TrajectorySample & TrajectorySE3Cubic::operator()(double)
    {
      return m_sample;
    }

    const TrajectorySample & TrajectorySE3Cubic::computeNext()
    {
      Eigen::Vector3d rot_diff_vec;
      Eigen::Vector3d cubic_tra;
      Eigen::Vector3d cubic_rot_tra;
      Eigen::Matrix3d cubic_rot;
      m_sample.resize(12, 6);

      Eigen::Matrix3d rot_diff = (m_init.rotation().transpose() * m_goal.rotation()).log();
      
      typedef Eigen::Matrix<double, 9, 1> Vector9;
      if (m_time < m_stime) {
        m_sample.pos.head<3>() = m_init.translation();
        m_sample.pos.tail<9>() = Eigen::Map<const Vector9>(&m_init.rotation()(0), 9);

        return m_sample;
      }
      else if (m_time > m_stime + m_duration) {
        m_sample.pos.head<3>() = m_goal.translation();
        m_sample.pos.tail<9>() = Eigen::Map<const Vector9>(&m_goal.rotation()(0), 9);

        return m_sample;
      }
      else {
        double a0, a1, a2, a3;
        double tau = this->cubic(m_time, m_stime, m_stime + m_duration, 0, 1, 0, 0);

        for (int i = 0; i < 3; i++) {
          a0 = m_init.translation()(i);
          a1 = 0.0; //m_init.vel(i);
          a2 = 3.0 / pow(m_duration, 2) * (m_goal.translation()(i) - m_init.translation()(i));
          a3 = -1.0 * 2.0 / pow(m_duration, 3) * (m_goal.translation()(i) - m_init.translation()(i));

          cubic_tra(i) = a0 + a1 * (m_time - m_stime) + a2 * pow(m_time - m_stime, 2) + a3 * pow(m_time - m_stime, 3);
        }

        m_cubic.rotation() = m_init.rotation() * (rot_diff * tau).exp();
        m_cubic.translation() = cubic_tra;
      
        m_sample.pos.head<3>() = m_cubic.translation();
        m_sample.pos.tail<9>() = Eigen::Map<const Vector9>(&m_cubic.rotation()(0), 9);

        m_sample.pos.head<3>() = m_cubic.translation();
        m_sample.pos.tail<9>() = Eigen::Map<const Vector9>(&m_cubic.rotation()(0), 9);

        return m_sample;
      }
    }

    void TrajectorySE3Cubic::getLastSample(TrajectorySample & sample) const
    {
      sample = m_sample;
    }

    bool TrajectorySE3Cubic::has_trajectory_ended() const
    {
      return true;
    }

    void TrajectorySE3Cubic::setGoalSample(const SE3 & goal_M)
    {
      m_goal = goal_M;
      this->setReference(m_goal);
    }
    void TrajectorySE3Cubic::setInitSample(const SE3 & init_M)
    {
      m_init = init_M;
    }
    void TrajectorySE3Cubic::setDuration(const double & duration)
    {
      m_duration = duration;
    }
    void TrajectorySE3Cubic::setCurrentTime(const double & time)
    {
      m_time = time;
    }
    void TrajectorySE3Cubic::setStartTime(const double & time)
    {
      m_stime = time;
    }

    void TrajectorySE3Cubic::setReference(const SE3 & ref) {
      m_sample.resize(12, 6);
      m_sample.pos.head<3>() = ref.translation();
      typedef Eigen::Matrix<double, 9, 1> Vector9;
      m_sample.pos.tail<9>() = Eigen::Map<const Vector9>(&ref.rotation()(0), 9);
    }

    const std::vector<Eigen::VectorXd> & TrajectorySE3Cubic::getWholeTrajectory(){
      traj_.clear();
      double time = m_stime;
      while (time <= m_stime + m_duration){
        this->setCurrentTime(time);
        this->computeNext();
        traj_.push_back(m_sample.pos);
        time += 0.001;
      }

      return traj_;
    }



  }

  
}
