#include "rci_qp_controller/formulation/inverse_dynamics_formulation_acc.hpp"

using namespace qp_controller;
using namespace qp_controller::math;
using namespace qp_controller::tasks;
using namespace qp_controller::solver;
using namespace std;

typedef pinocchio::Data Data;

InverseDynamicsFormulationAccForce::InverseDynamicsFormulationAccForce(const std::string & name, RobotWrapper & robot, bool verbose):
    InverseDynamicsFormulationBase(name, robot, verbose),
    m_data(robot.model()),
    m_solutionDecoded(false)
    {
      m_t = 0.0;
      m_v = robot.nv();
      m_k = 0;

      m_eq = 0;
      m_in = 0;
      m_hqpData.resize(2);
    //m_hqpData[0].push_back(solvers::make_pair<double, std::shared_ptr<ConstraintBase> >(1.0, m_baseDynamics));
    }


Data & InverseDynamicsFormulationAccForce::data()
{
  return m_data;
}

unsigned int InverseDynamicsFormulationAccForce::nVar() const
{
  return m_v+m_k;
}

unsigned int InverseDynamicsFormulationAccForce::nEq() const
{
  return m_eq;
}

unsigned int InverseDynamicsFormulationAccForce::nIn() const
{
  return m_in;
}


void InverseDynamicsFormulationAccForce::resizeHqpData()
{
  for(HQPData::iterator it=m_hqpData.begin(); it!=m_hqpData.end(); it++)
  {
    for(ConstraintLevel::iterator itt=it->begin(); itt!=it->end(); itt++)
    {
      itt->second->resize(itt->second->rows(), m_v+m_k);
    }
  }
}

template<class TaskLevelPointer>
void InverseDynamicsFormulationAccForce::addTask(TaskLevelPointer tl, double weight, unsigned int priorityLevel)
{
  if(priorityLevel > m_hqpData.size())
    m_hqpData.resize(priorityLevel);
  const ConstraintBase & c = tl->task.getConstraint();
  if(c.isEquality())
  {
    tl->constraint = std::make_shared<ConstraintEquality>(c.name(), c.rows(), m_v+m_k);
    if(priorityLevel==0)
      m_eq += c.rows();
  }
  else //if(c.isInequality())
  {
    tl->constraint = std::make_shared<ConstraintInequality>(c.name(), c.rows(), m_v+m_k);
    if(priorityLevel==0)
      m_in += c.rows();
  }
  m_hqpData[priorityLevel].push_back(make_pair<double, std::shared_ptr<ConstraintBase> >(weight, tl->constraint));
}


bool InverseDynamicsFormulationAccForce::addMotionTask(TaskMotion & taskmotion, double weight, unsigned int priorityLevel, double transition_duration)
{
  assert(weight>=0.0);
  assert(transition_duration>=0.0);
  
  // This part is not used frequently so we can do some tests.
  if (weight<0.0)
    std::cerr << __FILE__ <<  " " << __LINE__ << " "
      << "weight should be positive" << std::endl;

  // This part is not used frequently so we can do some tests.
  if (transition_duration<0.0)
    std::cerr << "transition_duration should be positive" << std::endl;

  auto tl = std::make_shared<TaskLevel>(taskmotion, priorityLevel);
  m_taskMotions.push_back(tl);
  addTask(tl, weight, priorityLevel);

  return true;
}

bool InverseDynamicsFormulationAccForce::updateTaskWeight(const std::string & task_name,
                                                          double weight)
{
  ConstraintLevel::iterator it;
  // do not look into first priority level because weights do not matter there
  for(unsigned int i=1; i<m_hqpData.size(); i++)
  {
    for(it=m_hqpData[i].begin(); it!=m_hqpData[i].end(); it++)
    {
      if(it->second->name() == task_name)
      {
        it->first = weight;
        return true;
      }
    }
  }
  return false;
}

const HQPData & InverseDynamicsFormulationAccForce::computeProblemData(double time, ConstRefVector q, ConstRefVector v)
{
  m_t = time;
  
  m_robot.computeAllTerms(m_data, q, v);

  for (auto& it : m_taskMotions)
  {
    const ConstraintBase & c = it->task.compute(time, q, v, m_data);
    
    if(c.isEquality())
    {
      it->constraint->matrix().leftCols(m_v) = c.matrix();
      it->constraint->vector() = c.vector();
    }
    else if(c.isInequality())
    {
      it->constraint->matrix().leftCols(m_v) = c.matrix();
      it->constraint->lowerBound() = c.lowerBound();
      it->constraint->upperBound() = c.upperBound();
    }
    else
    {
      it->constraint->matrix().leftCols(m_v) = Matrix::Identity(m_v, m_v);
      it->constraint->lowerBound() = c.lowerBound();
      it->constraint->upperBound() = c.upperBound();
    }
  }

  m_solutionDecoded = false;
  
  return m_hqpData;
}



bool InverseDynamicsFormulationAccForce::decodeSolution(const HQPOutput & sol)
{
  if(m_solutionDecoded)
    return true;

  const Matrix & M_a = m_robot.mass(m_data);
  const Vector & h_a = m_robot.nonLinearEffects(m_data);
  const Matrix & J_a = m_Jc.rightCols(m_v);
  
  m_dv = sol.x.head(m_v);
  m_f = sol.x.tail(m_k);
  
  m_tau = h_a;
  m_tau.noalias() += M_a * m_dv;
// m_tau.noalias() -= J_a.transpose() * m_f;
  m_solutionDecoded = true;
  return true;
  
}

const Vector & InverseDynamicsFormulationAccForce::getAccelerations(const HQPOutput & sol)
{
  decodeSolution(sol);
  return m_dv;
}

const Vector & InverseDynamicsFormulationAccForce::getActuatorForces(const HQPOutput & sol)
{
  decodeSolution(sol);
  return m_tau;
}

bool InverseDynamicsFormulationAccForce::removeTask(const std::string & taskName,
                                                    double )
{
#ifndef NDEBUG
  bool taskFound = removeFromHqpData(taskName);
  //assert(taskFound);
#else
  removeFromHqpData(taskName);
#endif
  
  for(auto it=m_taskMotions.begin(); it!=m_taskMotions.end(); it++)
  {
    if((*it)->task.name()==taskName)
    {
      if((*it)->priority==0)
      {
        if((*it)->constraint->isEquality())
          m_eq -= (*it)->constraint->rows();
        else if((*it)->constraint->isInequality())
          m_in -= (*it)->constraint->rows();
      }
      m_taskMotions.erase(it);
      return true;
    }
  }
 
  return false;
}
void InverseDynamicsFormulationAccForce::resetHqpData(){
  std::vector<std::shared_ptr<TaskLevel> > new_tasks;
  m_taskMotions = new_tasks;
  HQPData new_Data;
  m_hqpData = new_Data;
  m_hqpData.resize(2);
  m_eq = 0;
  m_in = 0;      
}

bool InverseDynamicsFormulationAccForce::removeFromHqpData(const std::string & name)
{
  bool found = false;
  for(HQPData::iterator it=m_hqpData.begin(); !found && it!=m_hqpData.end(); it++)
  {
    for(ConstraintLevel::iterator itt=it->begin(); !found && itt!=it->end(); itt++)
    {
      if(itt->second->name()==name)
      {
        it->erase(itt);
        return true;
      }
    }
  }
  return false;
}

