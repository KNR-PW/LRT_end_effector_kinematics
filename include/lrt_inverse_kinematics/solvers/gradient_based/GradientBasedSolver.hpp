#ifndef __LRT_GRADIENT_BASED_SOLVER__
#define __LRT_GRADIENT_BASED_SOLVER__


#include <Eigen/Dense>

#include <lrt_inverse_kinematics/solvers/InverseSolverInterface.hpp>

namespace lrt_inverse_kinematics
{

  class GradientBasedSolver: public InverseSolverInterface
  {

    public:
    
    GradientBasedSolver(ocs2::PinocchioInterface& pinocchioInterface,
      const IKModelInternalInfo& modelInternalInfo, const IKSolverInfo& solverInfo);

    bool getJointDeltas(const Eigen::VectorXd& actualJointPositions, 
      const Eigen::VectorXd& error,
      const std::vector<Eigen::Vector3d>& endEffectorPositions,
      const std::vector<pinocchio::SE3>& endEffectorTransforms,
      Eigen::VectorXd& jointDeltas) override final;

    virtual Eigen::MatrixXd getGradient(const Eigen::VectorXd& actualJointPositions,
      const std::vector<Eigen::Vector3d>& endEffectorPositions,
      const std::vector<pinocchio::SE3>& endEffectorTransforms) = 0;

    SolverType getSolverType() override final;

    virtual ~GradientBasedSolver() = default;

  };

};

#endif