#include <lrt_inverse_kinematics/solvers/InverseSolverInterface.hpp>
#include <lrt_inverse_kinematics/solvers/gradient_based/GradientBasedSolver.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

namespace lrt_inverse_kinematics
{

  class NewtonRaphsonSolver: public GradientBasedSolver
  {
    public:

    NewtonRaphsonSolver(ocs2::PinocchioInterface& pinocchioInterface,
      IKModelInfo& modelInfo, const IKSolverInfo& solverInfo);

    Eigen::MatrixXd getGradient(const Eigen::VectorXd& actualJointPositions) override final;

    const std::string& getSolverName() override final;


  };

};