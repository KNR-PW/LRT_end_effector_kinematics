#ifndef __LRT_NEWTON_RAPHSON_SOLVER__
#define __LRT_NEWTON_RAPHSON_SOLVER__


#include <lrt_inverse_kinematics/solvers/InverseSolverInterface.hpp>
#include <lrt_inverse_kinematics/solvers/gradient_based/GradientBasedSolver.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace lrt_inverse_kinematics
{

  class NewtonRaphsonSolver: public GradientBasedSolver
  {
    public:

    NewtonRaphsonSolver(ocs2::PinocchioInterface& pinocchioInterface,
      const IKModelInfo& modelInfo, const IKSolverInfo& solverInfo);

    Eigen::MatrixXd getGradient(const Eigen::VectorXd& actualJointPositions,
      const std::vector<Eigen::Vector3d>& endEffectorPositions,
      const std::vector<pinocchio::SE3>& endEffectorTransforms) override final;

    const std::string& getSolverName() override final;

  };

};

#endif