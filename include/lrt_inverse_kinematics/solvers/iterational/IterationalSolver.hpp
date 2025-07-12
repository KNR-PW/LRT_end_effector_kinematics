
#include <Eigen/Dense>

#include <lrt_inverse_kinematics/solvers/InverseSolverInterface.hpp>

namespace lrt_inverse_kinematics
{

  class IterationalSolver: public InverseSolverInterface
  {

    public:
    
    IterationalSolver(ocs2::PinocchioInterface& pinocchioInterface,
      IKModelInfo& modelInfo, const IKSolverInfo& solverInfo);

    bool getJointDeltas(const Eigen::VectorXd& actualJointPositions, 
      const Eigen::VectorXd& error,
      Eigen::VectorXd& jointDeltas) override final;

    virtual Eigen::MatrixXd getGradient(const Eigen::VectorXd& actualJointPositions) = 0;

    SolverType getSolverType() override final;

    virtual ~IterationalSolver() = default;

  };

};