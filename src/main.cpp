#include <multi_end_effector_kinematics/Settings.hpp>
#include <multi_end_effector_kinematics/InverseKinematics.hpp>
#include <multi_end_effector_kinematics/InverseKinematicsTest.hpp>
#include <multi_end_effector_kinematics/path_management/package_path.h>
#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <chrono>

int main(int argc, char** argv)
{
  // std::string urdfPathName = multi_end_effector_kinematics::package_path::getPath();
  // urdfPathName += "/../install/multi_end_effector_kinematics/share/multi_end_effector_kinematics/models/r6bot/r6bot.urdf";
  // std::string baseLinkName = "trunk_link";
  // std::string rightForwardFeet = "RFF_link";
  // std::string leftForwardFeet = "LFF_link";
  // std::string rightRearFeet = "RRF_link";
  // std::string leftRearFeet = "LRF_link";
  // std::vector<std::string> threeDofLinks{rightForwardFeet, rightRearFeet, leftForwardFeet, leftRearFeet};
  // std::vector<std::string> sixDofLinks;
  // std::string solverName = "NewtonRaphson";
  // multi_end_effector_kinematics::InverseSolverSettings solverSettings;
  // multi_end_effector_kinematics::InverseKinematicsTest inverseKinematicsTest(urdfPathName,
  // baseLinkName, threeDofLinks, sixDofLinks, solverSettings, solverName);

  // ocs2::PinocchioInterface* pinocchioInterface = inverseKinematicsTest.getPinocchioInterface();
  // multi_end_effector_kinematics::KinematicsModelSettings modelSettings = inverseKinematicsTest.getModelInfo();
  // multi_end_effector_kinematics::NewtonRaphsonSolver solver(*pinocchioInterface, modelSettings, solverSettings);

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






  // std::string urdfPathName = multi_end_effector_kinematics::package_path::getPath();
  // urdfPathName += "/../install/multi_end_effector_kinematics/share/multi_end_effector_kinematics/models/r6bot/r6bot.urdf";
  
  // std::string baseLinkName = "base_link";
  // std::string solverName = "NewtonRaphson";
  // std::vector<std::string> threeDofLinks{};
  // std::vector<std::string> sixDofLinks{"tool0"};
  // multi_end_effector_kinematics::InverseSolverSettings solverSettings;
  // solverSettings.dampingCoefficient = 1e-3;
  // solverSettings.stepCoefficient = 0.005;
  // solverSettings.maxIterations = 1000;
  // solverSettings.tolerance = 1e-4;

  // multi_end_effector_kinematics::KinematicsModelSettings modelSettings;
  // modelSettings.baseLinkName = baseLinkName;
  // modelSettings.threeDofEndEffectorNames = threeDofLinks;
  // modelSettings.sixDofEndEffectorNames = sixDofLinks;

  // multi_end_effector_kinematics::InverseKinematicsTest inverseKinematicsTest(urdfPathName,
  //   modelSettings, solverSettings, solverName);

  // ocs2::PinocchioInterface* pinocchioInterface = inverseKinematicsTest.getPinocchioInterface();
  // multi_end_effector_kinematics::KinematicsInternalModelSettings modelInternalInfo = inverseKinematicsTest.getModelInternalInfo();
  // multi_end_effector_kinematics::NewtonRaphsonSolver solver(*pinocchioInterface, modelInternalInfo, solverSettings);

  // pinocchio::Model modelTrue;
  // pinocchio::urdf::buildModel(urdfPathName, modelTrue, true);

  // pinocchio::Data dataTrue(modelTrue);

  // Eigen::VectorXd qTrue = pinocchio::neutral(modelTrue);
  // Eigen::VectorXd qInv = qTrue;
  // Eigen::VectorXd qInvNew = qTrue;
  
  // Eigen::Vector3d rpy = Eigen::Vector3d::Random() * M_PI_2 / 2;
  // Eigen::Vector3d position = Eigen::Vector3d::Random() * 0.25;
  // Eigen::Quaterniond quaterion = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitZ())
  // * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
  // * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitX());
  // pinocchio::SE3 sixDofPostion = pinocchio::SE3(quaterion, position);

  // pinocchio::SE3 sixDofPostionTest(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.15, 1.0, 0.7));

  // std::vector<pinocchio::SE3> sixDofPositions;
  // std::vector<Eigen::Vector3d> threeDofPositions;
  // sixDofPositions.push_back(sixDofPostionTest);

  // const double eps = 1e-4;
  // const int IT_MAX = 1000;
  // const double DT = 0.005;
  // const double damp = 1e-3;

  // size_t frameIndex = modelTrue.getFrameId("tool0");

  // inverseKinematicsTest.calculateJointPositions(qInv, threeDofPositions, sixDofPositions, qInvNew);

  // std::cout << "Convergence achieved!" << std::endl;
  // std::cout << qInvNew.transpose() << std::endl;
 
  // pinocchio::Data::Matrix6x J(6, modelTrue.nv);
  // J.setZero();
 
  // bool success = false;
  // typedef Eigen::Matrix<double, 6, 1> Vector6d;
  // Vector6d err;
  // Eigen::VectorXd v(modelTrue.nv);
  // for (int i = 0;; i++)
  // {
  //   pinocchio::forwardKinematics(modelTrue, dataTrue, qTrue);
  //   pinocchio::updateFramePlacements(modelTrue, dataTrue);
  //   const pinocchio::SE3 iMd = dataTrue.oMf[frameIndex].actInv(sixDofPostionTest);
  //   err = pinocchio::log6(iMd).toVector(); // in joint frame
  //   if (err.norm() < eps)
  //   {
  //     success = true;
  //     break;
  //   }
  //   if (i >= IT_MAX)
  //   {
  //     success = false;
  //     break;
  //   }
  //   pinocchio::computeFrameJacobian(modelTrue, dataTrue, qTrue, frameIndex, J); // J in joint frame
  //   pinocchio::Data::Matrix6 Jlog;
  //   pinocchio::Jlog6(iMd.inverse(), Jlog);
  //   J = -Jlog * J;
  //   pinocchio::Data::Matrix6 JJt;
  //   JJt.noalias() = J * J.transpose();
  //   JJt.diagonal().array() += damp;
  //   v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
  //   qTrue = pinocchio::integrate(modelTrue, qTrue, v * DT);
  // }

  // if (success)
  // {
  //   std::cout << "Convergence achieved true!" << std::endl;
  //   std::cout << qTrue.transpose() << std::endl;
  // }
  // else
  // {
  //   std::cout
  //     << "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
  //     << std::endl;
  //     std::cout << qTrue.transpose() << std::endl;
  //     std::cout << err.norm() << std::endl;
  // }



  std::string urdfPathName = multi_end_effector_kinematics::package_path::getPath();
  urdfPathName += "/../install/multi_end_effector_kinematics/share/multi_end_effector_kinematics/models/r6bot/r6bot.urdf";
  
  std::string baseLinkName = "base_link";
  std::string solverName = "NewtonRaphson";
  std::vector<std::string> threeDofLinks{};
  std::vector<std::string> sixDofLinks{"tool0"};
  multi_end_effector_kinematics::InverseSolverSettings solverSettings;
  solverSettings.dampingCoefficient = 1e-3;
  solverSettings.stepCoefficient = 0.005;
  solverSettings.maxIterations = 1000;
  solverSettings.tolerance = 1e-4;

  multi_end_effector_kinematics::KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLinkName;
  modelSettings.threeDofEndEffectorNames = threeDofLinks;
  modelSettings.sixDofEndEffectorNames = sixDofLinks;

  multi_end_effector_kinematics::InverseKinematicsTest inverseKinematicsTest(urdfPathName,
    modelSettings, solverSettings, solverName);

  ocs2::PinocchioInterface* pinocchioInterface = inverseKinematicsTest.getPinocchioInterface();
  const auto& model = pinocchioInterface->getModel();
  auto& data = pinocchioInterface->getData();
  multi_end_effector_kinematics::KinematicsInternalModelSettings modelInternalInfo = inverseKinematicsTest.getModelInternalInfo();
  multi_end_effector_kinematics::NewtonRaphsonSolver solver(*pinocchioInterface, modelInternalInfo, solverSettings);
  
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

  size_t frameIndex = model.getFrameId("tool0");

  const int ITER_ITER = 1'000;

  std::vector<int> times;

  for(int j = 0; j < ITER_ITER; ++j)
  {
    Eigen::VectorXd q = pinocchio::neutral(model);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    const int ITER = 1'000;

    Eigen::VectorXd jointDeltas(model.nv);

    for(int i = 0; i < ITER; ++i)
    {
      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      const pinocchio::SE3 iMd = data.oMf[frameIndex].actInv(sixDofPostionTest);
      Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector(); // in joint frame
      solver.getJointDeltas(q, err, threeDofPositions, sixDofPositions, jointDeltas);

      q += jointDeltas;
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    times.push_back(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count());

  }

  int mean = 0;

  for(int i = 0; i < ITER_ITER; ++i)
  {
    mean += times[i];
  }

  mean = mean / ITER_ITER;

  int sd = 0;

  for(int i = 0; i < ITER_ITER; ++i)
  {
    sd += (times[i] - mean) * (times[i] - mean);
  }

  sd = std::sqrt(sd / ITER_ITER);

  std::cout << "Time: " << mean << " +- " << sd << "[us]" << std::endl;


  return 0;
}