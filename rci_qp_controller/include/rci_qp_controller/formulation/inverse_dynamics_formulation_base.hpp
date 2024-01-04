#ifndef __inv_dyn_base_hpp__
#define __inv_dyn_base_hpp__

#include "rci_qp_controller/math/fwd.hpp"
#include "rci_qp_controller/robot/robot_wrapper.hpp"
#include "rci_qp_controller/tasks/task_motion.hpp"
#include "rci_qp_controller/solver/solver_HQP_base.hpp"

#include <string>

namespace qp_controller{
    struct TaskLevel
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        tasks::TaskBase & task;
        std::shared_ptr<math::ConstraintBase> constraint;
        unsigned int priority;

        TaskLevel(tasks::TaskBase & t, unsigned int priority);
    };

    class InverseDynamicsFormulationBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef pinocchio::Data Data;
        typedef math::Vector Vector;
        typedef math::RefVector RefVector;
        typedef math::ConstRefVector ConstRefVector;
        typedef tasks::TaskMotion TaskMotion;
        typedef tasks::TaskBase TaskBase;
        typedef solver::HQPData HQPData;
        typedef solver::HQPOutput HQPOutput;
        typedef robot::RobotWrapper RobotWrapper;

        InverseDynamicsFormulationBase(const std::string & name, RobotWrapper & robot, bool verbose=false);

       
    protected:
        std::string m_name;
        RobotWrapper m_robot;
        bool m_verbose;
    };
    
}

#endif