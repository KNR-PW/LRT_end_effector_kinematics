#include <gtest/gtest.h>
#include <lrt_inverse_kinematics/solvers/gradient_based/NewtonRaphsonSolver.hpp>
#include <lrt_inverse_kinematics/InverseKinematicsTest.hpp>
#include <lrt_inverse_kinematics/path_management/package_path.h>

#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Geometry>

using namespace lrt_inverse_kinematics;


static constexpr ocs2::scalar_t tolerance = 1e-6;
static constexpr size_t numTests = 100;

TEST(NewtonRaphsonTest, threeDofGradient)
{
  std::string urdfPathName = package_path::getPath();
  urdfPathName += "/../install/lrt_inverse_kinematics/share/lrt_inverse_kinematics/models/r6bot/meldog_no_base_link.urdf";
  
  std::string baseLinkName = "trunk_link";
  std::string rightForwardFeet = "RFF_link";
  std::string leftForwardFeet = "LFF_link";
  std::string rightRearFeet = "RRF_link";
  std::string leftRearFeet = "LRF_link";
  std::vector<std::string> threeDofLinks{rightForwardFeet, rightRearFeet, leftForwardFeet, leftRearFeet};
  std::vector<std::string> sixDofLinks;
  std::string solverName = "NewtonRaphson";
  IKSolverInfo solverInfo;
  InverseKinematicsTest inverseKinematicsTest(urdfPathName,
    baseLinkName, threeDofLinks, sixDofLinks, solverInfo, solverName);

  ocs2::PinocchioInterface* pinocchioInterface = inverseKinematicsTest.getPinocchioInterface();
  IKModelInfo modelInfo = inverseKinematicsTest.getModelInfo();
  lrt_inverse_kinematics::NewtonRaphsonSolver solver(*pinocchioInterface, modelInfo, solverInfo);

  const pinocchio::Model& modelInverse = pinocchioInterface->getModel();
  pinocchio::Data& dataInverse = pinocchioInterface->getData();

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

    pinocchio::forwardKinematics(modelInverse, dataInverse, q);
    Eigen::MatrixXd jacobianInverse = solver.getGradient(q, threeDofPositions, sixDofPositions);

    pinocchio::Data::Matrix6x jacobianTrue(6, modelTrue.nv);
    jacobianTrue.setZero();
    Eigen::Matrix<double, 6, 1> errorTrue;

    pinocchio::forwardKinematics(modelTrue, dataTrue, q);
    const pinocchio::SE3 fMd = dataTrue.oMf[endEffectorIndex].actInv(sixDofPostion);
    errorTrue = pinocchio::log6(fMd).toVector();

    pinocchio::computeFrameJacobian(modelTrue, dataTrue, q, endEffectorIndex, pinocchio::LOCAL, jacobianTrue);
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(fMd.inverse(), Jlog);
    jacobianTrue = -Jlog * jacobianTrue;

    EXPECT_TRUE(jacobianTrue.isApprox(jacobianInverse, tolerance));
  }

  
}

TEST(NewtonRaphsonTest, SixDofGradient)
{
  std::string urdfPathName = package_path::getPath();
  urdfPathName += "/../install/lrt_inverse_kinematics/share/lrt_inverse_kinematics/models/r6bot/r6bot.urdf";
  
  std::string baseLinkName = "base_link";
  std::string solverName = "NewtonRaphson";
  std::vector<std::string> threeDofLinks{};
  std::vector<std::string> sixDofLinks{"tool0"};
  IKSolverInfo solverInfo;
  InverseKinematicsTest inverseKinematicsTest(urdfPathName,
    baseLinkName, threeDofLinks, sixDofLinks, solverInfo, solverName);

  ocs2::PinocchioInterface* pinocchioInterface = inverseKinematicsTest.getPinocchioInterface();
  IKModelInfo modelInfo = inverseKinematicsTest.getModelInfo();
  lrt_inverse_kinematics::NewtonRaphsonSolver solver(*pinocchioInterface, modelInfo, solverInfo);

  const pinocchio::Model& modelInverse = pinocchioInterface->getModel();
  pinocchio::Data& dataInverse = pinocchioInterface->getData();

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

    pinocchio::forwardKinematics(modelInverse, dataInverse, q);
    Eigen::MatrixXd jacobianInverse = solver.getGradient(q, threeDofPositions, sixDofPositions);

    pinocchio::Data::Matrix6x jacobianTrue(6, modelTrue.nv);
    jacobianTrue.setZero();
    Eigen::Matrix<double, 6, 1> errorTrue;

    pinocchio::forwardKinematics(modelTrue, dataTrue, q);
    const pinocchio::SE3 fMd = dataTrue.oMf[endEffectorIndex].actInv(sixDofPostion);
    errorTrue = pinocchio::log6(fMd).toVector();

    pinocchio::computeFrameJacobian(modelTrue, dataTrue, q, endEffectorIndex, pinocchio::LOCAL, jacobianTrue);
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(fMd.inverse(), Jlog);
    jacobianTrue = -Jlog * jacobianTrue;

    EXPECT_TRUE(jacobianTrue.isApprox(jacobianInverse, tolerance));
  }
}