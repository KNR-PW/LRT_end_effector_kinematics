#include <gtest/gtest.h>
#include <multi_end_effector_kinematics/solvers/gradient_based/NewtonRaphsonSolver.hpp>
#include <multi_end_effector_kinematics/../../test/include/MultiEndEffectorKinematicsTest.hpp>
#include <multi_end_effector_kinematics/path_management/package_path.h>

#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Geometry>

using namespace multi_end_effector_kinematics;


static constexpr ocs2::scalar_t tolerance = 1e-6;
static constexpr size_t numTests = 100;

TEST(NewtonRaphsonTest, threeDofGradient)
{
  std::string urdfPathName = package_path::getPath();
  urdfPathName += "/../install/multi_end_effector_kinematics/share/multi_end_effector_kinematics/models/meldog/meldog_no_base_link.urdf";
  
  std::string baseLinkName = "trunk_link";
  std::string rightForwardFeet = "RFF_link";
  std::string leftForwardFeet = "LFF_link";
  std::string rightRearFeet = "RRF_link";
  std::string leftRearFeet = "LRF_link";
  std::vector<std::string> threeDofLinks{rightForwardFeet, rightRearFeet, leftForwardFeet, leftRearFeet};
  std::vector<std::string> sixDofLinks;
  std::string solverName = "NewtonRaphson";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLinkName;
  modelSettings.threeDofEndEffectorNames = threeDofLinks;

  InverseSolverSettings solverSettings;
  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface pinocchioInterface = kinematicsTest.getPinocchioInterface();
  const auto modelInternalSettings = kinematicsTest.getModelInternalSettings();
  multi_end_effector_kinematics::NewtonRaphsonSolver solver(pinocchioInterface, modelInternalSettings, solverSettings);

  const pinocchio::Model& modelInverse = pinocchioInterface.getModel();
  pinocchio::Data& dataInverse = pinocchioInterface.getData();

  pinocchio::Model modelTrue;
  pinocchio::urdf::buildModel(urdfPathName, modelTrue);

  pinocchio::Data dataTrue(modelTrue);

  std::vector<size_t> endEffectorIndexes;

  for(const auto& name: threeDofLinks)
  {
    endEffectorIndexes.push_back(modelTrue.getFrameId(name));
  }

  for(int i = 0; i < numTests; ++i)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Random(modelTrue.nq);
    
    Eigen::Vector3d position = Eigen::Vector3d::Random() * 0.5;

    std::vector<pinocchio::SE3> sixDofPositions;
    std::vector<Eigen::Vector3d> threeDofPositions{position, position, position, position};

    pinocchio::forwardKinematics(modelInverse, dataInverse, q);
    Eigen::MatrixXd jacobianInverse = solver.getGradient(q, threeDofPositions, sixDofPositions);

    Eigen::MatrixXd jacobianTrue(12, modelTrue.nv);
    pinocchio::Data::Matrix6x singleJacobian(6, modelTrue.nv);

    pinocchio::forwardKinematics(modelTrue, dataTrue, q);

    for(size_t i = 0; i < 4; ++i)
    {
      singleJacobian.setZero();
      pinocchio::computeFrameJacobian(modelTrue, dataTrue, q, endEffectorIndexes[i], pinocchio::LOCAL, singleJacobian);
      jacobianTrue.block(3 * i, 0, 3, modelTrue.nv) = -singleJacobian.block(0, 0, 3, modelTrue.nv);
    }

    EXPECT_TRUE(jacobianTrue.isApprox(jacobianInverse, tolerance));
  }
}

TEST(NewtonRaphsonTest, SixDofGradient)
{
  std::string urdfPathName = package_path::getPath();
  urdfPathName += "/../install/multi_end_effector_kinematics/share/multi_end_effector_kinematics/models/r6bot/r6bot.urdf";
  
  std::string baseLinkName = "world";
  std::string solverName = "NewtonRaphson";
  std::vector<std::string> threeDofLinks{};
  std::vector<std::string> sixDofLinks{"tool0"};

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLinkName;
  modelSettings.sixDofEndEffectorNames = sixDofLinks;

  InverseSolverSettings solverSettings;
  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface pinocchioInterface = kinematicsTest.getPinocchioInterface();
  const auto modelInternalSettings = kinematicsTest.getModelInternalSettings();
  multi_end_effector_kinematics::NewtonRaphsonSolver solver(pinocchioInterface, modelInternalSettings, solverSettings);

  const pinocchio::Model& modelInverse = pinocchioInterface.getModel();
  pinocchio::Data& dataInverse = pinocchioInterface.getData();

  pinocchio::Model modelTrue;
  pinocchio::urdf::buildModel(urdfPathName, modelTrue);

  pinocchio::Data dataTrue(modelTrue);

  size_t endEffectorIndex = modelTrue.getFrameId("tool0");

  for(int i = 0; i < numTests; ++i)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Random(modelTrue.nq);
  
    Eigen::Vector3d rpy = Eigen::Vector3d::Random() * M_PI_2;
    Eigen::Vector3d position = Eigen::Vector3d::Random() * 0.5;
    Eigen::Quaterniond quaterion = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitX());
    pinocchio::SE3 sixDofPostion = pinocchio::SE3(quaterion, position);

    std::vector<pinocchio::SE3> sixDofPositions;
    std::vector<Eigen::Vector3d> threeDofPositions;
    sixDofPositions.push_back(sixDofPostion);

    pinocchio::framesForwardKinematics(modelInverse, dataInverse, q);
    Eigen::MatrixXd jacobianInverse = solver.getGradient(q, threeDofPositions, sixDofPositions);

    pinocchio::Data::Matrix6x jacobianTrue(6, modelTrue.nv);
    jacobianTrue.setZero();

    pinocchio::framesForwardKinematics(modelTrue, dataTrue, q);
    const pinocchio::SE3 fMd = dataTrue.oMf[endEffectorIndex].actInv(sixDofPostion);

    pinocchio::computeFrameJacobian(modelTrue, dataTrue, q, endEffectorIndex, pinocchio::LOCAL, jacobianTrue);
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(fMd.inverse(), Jlog);
    jacobianTrue = -Jlog * jacobianTrue;

    EXPECT_TRUE(jacobianTrue.isApprox(jacobianInverse, tolerance));
  }
}