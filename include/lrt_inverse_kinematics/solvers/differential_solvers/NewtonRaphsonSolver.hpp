#include <lrt_inverse_kinematics/solvers/InvereSolverInterface.hpp>

namespace lrt_inverse_kinematics
{

  class NewtonRaphsonSolver: public DifferentialSolver
  {
    public:

    NewtonRaphsonSolver(IKModelInfo& modelInfo, const IKSolverInfo& solverInfo);

    Eigen::MatrixXd getGradient(const Eigen::VectorXd& actualJointPositions) override;
  };

};