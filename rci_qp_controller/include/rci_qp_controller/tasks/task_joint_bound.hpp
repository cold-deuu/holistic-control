#ifndef __task_joint_bound_hpp__
#define __task_joint_bound_hpp__

#include "rci_qp_controller/tasks/task_motion.hpp"
#include "rci_qp_controller/math/constraint_bound.hpp"

#include <pinocchio/multibody/fwd.hpp>

namespace qp_controller
{
  namespace tasks
  {
    class TaskTorqueBounds : public TaskMotion
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef math::Vector Vector;
        typedef math::ConstraintBound ConstraintBound;
        typedef pinocchio::Data Data;

        TaskTorqueBounds(const std::string & name,
                        RobotWrapper & robot);

        int dim() const;

        const ConstraintBase & compute(const double t,
                                        ConstRefVector q,
                                        ConstRefVector v,
                                        Data & data);

        const ConstraintBase & getConstraint() const;

        void setTorqueBounds(ConstRefVector lower, ConstRefVector upper);
        const Vector & getTorqueLowerBounds() const;
        const Vector & getTorqueUpperBounds() const;

        virtual void setMask(math::ConstRefVector mask);

        protected:
        Vector m_torque_lb, m_torque_ub;
        ConstraintBound m_constraint;
        int m_nv, m_na;
    };

class TaskJointBounds : public TaskMotion
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef math::Vector Vector;
        typedef math::ConstraintBound ConstraintBound;
        typedef pinocchio::Data Data;

        TaskJointBounds(const std::string & name,
                        RobotWrapper & robot);

        int dim() const;

        const ConstraintBase & compute(const double t,
                                        ConstRefVector q,
                                        ConstRefVector v,
                                        Data & data);

        const ConstraintBase & getConstraint() const;

        void setJointBounds(ConstRefVector lower, ConstRefVector upper);
        const Vector & getJointLowerBounds() const;
        const Vector & getJointUpperBounds() const;

        virtual void setMask(math::ConstRefVector mask);

        protected:
        Vector m_joint_lb, m_joint_ub;
        ConstraintBound m_constraint;
        int m_nv, m_na;
        double m_buffer;
    };
  }
}

#endif 