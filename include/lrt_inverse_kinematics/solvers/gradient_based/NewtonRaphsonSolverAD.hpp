
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <lrt_inverse_kinematics/solvers/InverseSolverInterface.hpp>
#include <lrt_inverse_kinematics/solvers/gradient_based/GradientBasedSolver.hpp>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

namespace lrt_inverse_kinematics
{

  class NewtonRaphsonSolverAD: public GradientBasedSolver
  {
    public:

    NewtonRaphsonSolverAD(ocs2::PinocchioInterface& pinocchioInterface,
      const IKModelInternalInfo& modelInternalInfo, const IKSolverInfo& solverInfo);

    Eigen::MatrixXd getGradient(const Eigen::VectorXd& actualJointPositions,
      const std::vector<Eigen::Vector3d>& endEffectorPositions,
      const std::vector<pinocchio::SE3>& endEffectorTransforms) override final;

    const std::string& getSolverName() override final;

    private:

    ocs2::ad_vector_t getErrorPositionsCppAd(
      ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
      const ocs2::ad_vector_t& actualJointPositions,
      const ocs2::ad_vector_t& jointVelocityDeltas,
      const ocs2::ad_vector_t& logEndEffectorTransforms);

    std::unique_ptr<ocs2::CppAdInterface> errorPositionsAdFunction_;

  };

};