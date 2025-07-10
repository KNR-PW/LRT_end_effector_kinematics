#ifndef __LRT_INVERSE_SOLVER_INTERFACE__
#define __LRT_INVERSE_SOLVER_INTERFACE__

#include <vector>
#include <Eigen/Dense>
#include <pinocchio/fwd>
#include <pinocchio/algorithm/kinematics.hpp>

#include <lrt_inverse_kinematics/InverseKinematicsInfo.hpp>


namespace lrt_inverse_kinematics
{
  enum class SolverType: uint8_t
  {
    DIFFERENTIAL = 0,
    OPTIMIZATION = 2,
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

    InverseSolverInterface(IKModelInfo& modelInfo, const IKSolverInfo& solverInfo);

    virtual Eigen::VectorXd getJointDeltas(const Eigen::VectorXd& actualJointPositions, 
                                           const VectorXd& error) = 0;

    SolverType getSolverType();
    
    virtual ~InverseSolverInterface() = default;

    private:

    IKModelInfo* modelInfo_;
    const IKSolverInfo* solverInfo_;

    SolverType solverType_;
    TaskType taskType_;

  };

};
#endif