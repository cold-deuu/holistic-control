#include "rci_qp_controller/formulation/inverse_dynamics_formulation_base.hpp"

namespace qp_controller{
    TaskLevel::TaskLevel(tasks::TaskBase & t, unsigned int priority):
    task(t),
    priority(priority)
    {}

  InverseDynamicsFormulationBase::InverseDynamicsFormulationBase(const std::string & name, RobotWrapper & robot, bool verbose)
  : m_name(name)
  , m_robot(robot)
  , m_verbose(verbose)
  {}

}
