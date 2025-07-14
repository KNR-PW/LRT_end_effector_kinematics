#ifndef __LRT_INVERSE_KINEMATICS_HPP__
#define __LRT_INVERSE_KINEMATICS_HPP__

#include <utility>
#include <memory>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <lrt_inverse_kinematics/InverseKinematicsInfo.hpp>
#include <lrt_inverse_kinematics/solvers/InverseSolverInterface.hpp>
#include <lrt_inverse_kinematics/solvers/SolversList.hpp>



namespace lrt_inverse_kinematics
{
  class InverseKinematics
  {

    public:

    InverseKinematics(const std::string urdfFilePath,
      const std::string baseLinkName,
      const std::vector<std::string>& threeDofEndEffectorNames,
      const std::vector<std::string>& sixDofEndEffectorNames,
      IKSolverInfo solverInfo,
      const std::string solverName);

    std::pair<bool, ReturnFlag> calculateJointDeltas(const Eigen::VectorXd& actualJointPositions, 
      const std::vector<Eigen::Vector3d>& endEffectorPositions,
      const std::vector<pinocchio::SE3>& endEffectorTransforms,
      Eigen::VectorXd& jointDeltas);

    std::pair<bool, ReturnFlag> calculateJointPositions(const Eigen::VectorXd& actualJointPositions, 
      const std::vector<Eigen::Vector3d>& endEffectorPositions,
      const std::vector<pinocchio::SE3>& endEffectorTransforms,
      Eigen::VectorXd& newJointPositions);

    
    bool checkPositionBounds(const Eigen::VectorXd& newJointPositions);

    bool checkVelocityBounds(const Eigen::VectorXd& jointDeltas);

    const IKModelInfo& getModelInfo();

    const IKSolverInfo& getSolverInfo();

    SolverType getSolverType();

    const std::string& getSolverName();

    TaskType getTaskType();



    private:

    Eigen::VectorXd getErrorPositions(const Eigen::VectorXd& actualJointPositions, 
      const std::vector<Eigen::Vector3d>& endEffectorPositions,
      const std::vector<pinocchio::SE3>& endEffectorTransforms);

    std::unique_ptr<InverseSolverInterface> getSolver(const std::string& solverName);

    
    std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterface_;

    IKModelInfo modelInfo_;
    IKSolverInfo solverInfo_;
    std::unique_ptr<InverseSolverInterface> solverImplementation_;
  };

}; // lrt_inverse_kinematics


#endif