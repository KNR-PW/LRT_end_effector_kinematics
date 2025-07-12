#ifndef __LRT_INVERSE_SOLVER_INTERFACE__
#define __LRT_INVERSE_SOLVER_INTERFACE__

#include <vector>
#include <stdexcept>

#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <lrt_inverse_kinematics/InverseKinematicsInfo.hpp>


namespace lrt_inverse_kinematics
{
  enum class SolverType: uint8_t
  {
    ITERATIONAL = 0,
    OPTIMIZATION = 1,
  };

  enum class TaskType: uint8_t
  {
    NORMAL = 0,
    REDUNDANT = 1,
    DAMPED = 2,
  };

  class InverseSolverInterface
  {

    public:

    InverseSolverInterface(ocs2::PinocchioInterface& pinocchioInterface,
      IKModelInfo& modelInfo, const IKSolverInfo& solverInfo);

    virtual bool getJointDeltas(const Eigen::VectorXd& actualJointPositions, 
      const Eigen::VectorXd& error,
      Eigen::VectorXd& jointDeltas) = 0;

    virtual SolverType getSolverType() = 0;

    virtual const std::string& getSolverName() = 0;

    TaskType getTaskType();
    
    virtual ~InverseSolverInterface() = default;

    protected:
    
    ocs2::PinocchioInterface* pinocchioInterface_;
    const IKModelInfo* modelInfo_;
    const IKSolverInfo* solverInfo_;
    
    std::string solverName_;
    SolverType solverType_;
    
    private:

    TaskType taskType_;

  };

};
#endif