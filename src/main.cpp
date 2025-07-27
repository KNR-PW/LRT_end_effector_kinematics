#include <lrt_inverse_kinematics/InverseKinematicsInfo.hpp>
#include <lrt_inverse_kinematics/InverseKinematics.hpp>
#include <lrt_inverse_kinematics/InverseKinematicsTest.hpp>
#include <lrt_inverse_kinematics/path_management/package_path.h>
#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

int main(int argc, char** argv)
{
  // std::string urdfPathName = lrt_inverse_kinematics::package_path::getPath();
  // urdfPathName += "/../install/lrt_inverse_kinematics/share/lrt_inverse_kinematics/models/r6bot/r6bot.urdf";
  // std::string baseLinkName = "trunk_link";
  // std::string rightForwardFeet = "RFF_link";
  // std::string leftForwardFeet = "LFF_link";
  // std::string rightRearFeet = "RRF_link";
  // std::string leftRearFeet = "LRF_link";
  // std::vector<std::string> threeDofLinks{rightForwardFeet, rightRearFeet, leftForwardFeet, leftRearFeet};
  // std::vector<std::string> sixDofLinks;
  // std::string solverName = "NewtonRaphson";
  // lrt_inverse_kinematics::IKSolverInfo solverInfo;
  // lrt_inverse_kinematics::InverseKinematicsTest inverseKinematicsTest(urdfPathName,
  // baseLinkName, threeDofLinks, sixDofLinks, solverInfo, solverName);

  // ocs2::PinocchioInterface* pinocchioInterface = inverseKinematicsTest.getPinocchioInterface();
  // lrt_inverse_kinematics::IKModelInfo modelInfo = inverseKinematicsTest.getModelInfo();
  // lrt_inverse_kinematics::NewtonRaphsonSolver solver(*pinocchioInterface, modelInfo, solverInfo);

  // const pinocchio::Model& modelInverse = pinocchioInterface->getModel();
  // pinocchio::Data& dataInverse = pinocchioInterface->getData();

  // pinocchio::Model model;
  // pinocchio::urdf::buildModel(urdfPathName, model);

  // pinocchio::Data data(model);

  // Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq);

  // size_t frameIndex = model.getFrameId("tool0");

  // pinocchio::forwardKinematics(modelInverse, dataInverse, q);
  
  // Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  // Eigen::Vector3d position = Eigen::Vector3d(0.5, 0, 0);
  // pinocchio::SE3 testPostion = pinocchio::SE3(rotation, position);
  // std::vector<pinocchio::SE3> sixDofPositions;
  // std::vector<Eigen::Vector3d> threeDofPositions;
  // sixDofPositions.push_back(testPostion);

  // Eigen::MatrixXd jacobianInverse = solver.getGradient(q, threeDofPositions, sixDofPositions);

  // std::cout << jacobianInverse << std::endl;

  // pinocchio::computeJointJacobians(model, data, q);

  // Eigen::MatrixXd jacobianOne = pinocchio::getFrameJacobian(model, data, frameIndex, pinocchio::LOCAL);

  // Eigen::MatrixXd jacobianTwo(6, model.nv);

  // pinocchio::computeFrameJacobian(model, data, q, frameIndex, pinocchio::LOCAL, jacobianTwo);

  // std::cout << (jacobianOne - jacobianTwo).norm() << std::endl;

  std::string urdfPathName = lrt_inverse_kinematics::package_path::getPath();
  urdfPathName += "/../install/lrt_inverse_kinematics/share/lrt_inverse_kinematics/models/r6bot/r6bot.urdf";
  
  std::string baseLinkName = "base_link";
  std::string solverName = "NewtonRaphson";
  std::vector<std::string> threeDofLinks{};
  std::vector<std::string> sixDofLinks{"tool0"};
  lrt_inverse_kinematics::IKSolverInfo solverInfo;
  solverInfo.dampingCoefficient_ = 1e-3;
  solverInfo.stepCoefficient_ = 0.005;
  solverInfo.maxIterations_ = 1000;
  solverInfo.tolerance_ = 1e-4;

  lrt_inverse_kinematics::IKModelInfo modelInfo;
  modelInfo.baseLinkName_ = baseLinkName;
  modelInfo.threeDofEndEffectorNames_ = threeDofLinks;
  modelInfo.sixDofEndEffectorNames_ = sixDofLinks;

  lrt_inverse_kinematics::InverseKinematicsTest inverseKinematicsTest(urdfPathName,
    modelInfo, solverInfo, solverName);

  ocs2::PinocchioInterface* pinocchioInterface = inverseKinematicsTest.getPinocchioInterface();
  lrt_inverse_kinematics::IKModelInternalInfo modelInternalInfo = inverseKinematicsTest.getModelInternalInfo();
  lrt_inverse_kinematics::NewtonRaphsonSolver solver(*pinocchioInterface, modelInternalInfo, solverInfo);

  pinocchio::Model modelTrue;
  pinocchio::urdf::buildModel(urdfPathName, modelTrue, true);

  pinocchio::Data dataTrue(modelTrue);

  Eigen::VectorXd qTrue = pinocchio::neutral(modelTrue);
  Eigen::VectorXd qInv = qTrue;
  Eigen::VectorXd qInvNew = qTrue;
  
  Eigen::Vector3d rpy = Eigen::Vector3d::Random() * M_PI_2 / 2;
  Eigen::Vector3d position = Eigen::Vector3d::Random() * 0.25;
  Eigen::Quaterniond quaterion = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitZ())
  * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
  * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitX());
  pinocchio::SE3 sixDofPostion = pinocchio::SE3(quaterion, position);

  pinocchio::SE3 sixDofPostionTest(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.15, 1.0, 0.7));

  std::vector<pinocchio::SE3> sixDofPositions;
  std::vector<Eigen::Vector3d> threeDofPositions;
  sixDofPositions.push_back(sixDofPostionTest);

  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 0.005;
  const double damp = 1e-3;

  size_t frameIndex = modelTrue.getFrameId("tool0");

  inverseKinematicsTest.calculateJointPositions(qInv, threeDofPositions, sixDofPositions, qInvNew);

  std::cout << "Convergence achieved!" << std::endl;
  std::cout << qInvNew.transpose() << std::endl;
 
  pinocchio::Data::Matrix6x J(6, modelTrue.nv);
  J.setZero();
 
  bool success = false;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;
  Eigen::VectorXd v(modelTrue.nv);
  for (int i = 0;; i++)
  {
    pinocchio::forwardKinematics(modelTrue, dataTrue, qTrue);
    pinocchio::updateFramePlacements(modelTrue, dataTrue);
    const pinocchio::SE3 iMd = dataTrue.oMf[frameIndex].actInv(sixDofPostionTest);
    err = pinocchio::log6(iMd).toVector(); // in joint frame
    if (err.norm() < eps)
    {
      success = true;
      break;
    }
    if (i >= IT_MAX)
    {
      success = false;
      break;
    }
    pinocchio::computeFrameJacobian(modelTrue, dataTrue, qTrue, frameIndex, J); // J in joint frame
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(iMd.inverse(), Jlog);
    J = -Jlog * J;
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    qTrue = pinocchio::integrate(modelTrue, qTrue, v * DT);
  }

  if (success)
  {
    std::cout << "Convergence achieved true!" << std::endl;
    std::cout << qTrue.transpose() << std::endl;
  }
  else
  {
    std::cout
      << "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
      << std::endl;
      std::cout << qTrue.transpose() << std::endl;
      std::cout << err.norm() << std::endl;
  }
}