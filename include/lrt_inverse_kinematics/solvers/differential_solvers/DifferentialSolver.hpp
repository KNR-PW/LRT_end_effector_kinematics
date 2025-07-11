
#include <Eigen/Dense>

#include <lrt_inverse_kinematics/solvers/InverseSolverInterface.hpp>

namespace lrt_inverse_kinematics
{

  class DifferentialSolver: public InverseSolverInterface
  {

    public:
    DifferentialSolver(IKModelInfo& modelInfo, const IKSolverInfo& solverInfo);

    Eigen::VectorXd getJointDeltas(const Eigen::VectorXd& actualJointPositions, 
                                   const Eigen::VectorXd& error) override final;

    virtual Eigen::MatrixXd getGradient(const Eigen::VectorXd& actualJointPositions) = 0;

    virtual ~DifferentialSolver() = 0;

  };

};