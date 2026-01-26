#include <benchmark/benchmark.h>
#include <multi_end_effector_kinematics/path_management/package_path.h>
#include <multi_end_effector_kinematics/../../test/include/MultiEndEffectorKinematicsTest.hpp>

using namespace multi_end_effector_kinematics;


static void classicCalculateJointPositionsThreeDoFsSmallChange(benchmark::State &state) 
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
  solverSettings.dampingCoefficient = 1e-6;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;
  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface pinocchioInterface = kinematicsTest.getPinocchioInterface();

  const pinocchio::Model& model = pinocchioInterface.getModel();
  pinocchio::Data& data = pinocchioInterface.getData();
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq) * M_PI_2;
    Eigen::VectorXd dq = Eigen::VectorXd::Random(model.nq);
    pinocchio::framesForwardKinematics(model, data, q + dq);

    std::vector<Eigen::Vector3d> threeDofPositions;

    std::vector<size_t> endEffectorIndexes;

    for(const auto& name: threeDofLinks)
    {
      endEffectorIndexes.push_back(model.getFrameId(name));
    }

    for(size_t i = 0; i < 4; ++i)
    {
      threeDofPositions.push_back(data.oMf[endEffectorIndexes[i]].translation());
    }

    Eigen::VectorXd qInverse;
    for (auto _ : state)
      benchmark::DoNotOptimize(kinematicsTest.calculateJointPositions(q, threeDofPositions, 
        qInverse));
}

static void classicCalculateJointPositionsThreeDoFsBigChange(benchmark::State &state) 
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
  solverSettings.dampingCoefficient = 1e-6;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;
  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface pinocchioInterface = kinematicsTest.getPinocchioInterface();

  const pinocchio::Model& model = pinocchioInterface.getModel();
  pinocchio::Data& data = pinocchioInterface.getData();
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq) * M_PI_2;
    Eigen::VectorXd dq = Eigen::VectorXd::Random(model.nq) * M_PI;
    pinocchio::framesForwardKinematics(model, data, q + dq);

    std::vector<Eigen::Vector3d> threeDofPositions;

    std::vector<size_t> endEffectorIndexes;

    for(const auto& name: threeDofLinks)
    {
      endEffectorIndexes.push_back(model.getFrameId(name));
    }

    for(size_t i = 0; i < 4; ++i)
    {
      threeDofPositions.push_back(data.oMf[endEffectorIndexes[i]].translation());
    }

    Eigen::VectorXd qInverse;
    for (auto _ : state)
      benchmark::DoNotOptimize(kinematicsTest.calculateJointPositions(q, threeDofPositions, 
        qInverse));
}

static void ADCalculateJointPositionsThreeDoFsSmallChange(benchmark::State &state) 
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
  std::string solverName = "NewtonRaphsonAD";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLinkName;
  modelSettings.threeDofEndEffectorNames = threeDofLinks;

  InverseSolverSettings solverSettings;
  solverSettings.dampingCoefficient = 1e-6;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;
  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface pinocchioInterface = kinematicsTest.getPinocchioInterface();

  const pinocchio::Model& model = pinocchioInterface.getModel();
  pinocchio::Data& data = pinocchioInterface.getData();
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq) * M_PI_2;
    Eigen::VectorXd dq = Eigen::VectorXd::Random(model.nq);
    pinocchio::framesForwardKinematics(model, data, q + dq);

    std::vector<Eigen::Vector3d> threeDofPositions;

    std::vector<size_t> endEffectorIndexes;

    for(const auto& name: threeDofLinks)
    {
      endEffectorIndexes.push_back(model.getFrameId(name));
    }

    for(size_t i = 0; i < 4; ++i)
    {
      threeDofPositions.push_back(data.oMf[endEffectorIndexes[i]].translation());
    }

    Eigen::VectorXd qInverse;
    for (auto _ : state)
      benchmark::DoNotOptimize(kinematicsTest.calculateJointPositions(q, threeDofPositions, 
        qInverse));
}

static void ADCalculateJointPositionsThreeDoFsBigChange(benchmark::State &state) 
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
  std::string solverName = "NewtonRaphsonAD";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLinkName;
  modelSettings.threeDofEndEffectorNames = threeDofLinks;

  InverseSolverSettings solverSettings;
  solverSettings.dampingCoefficient = 1e-6;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;
  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface pinocchioInterface = kinematicsTest.getPinocchioInterface();

  const pinocchio::Model& model = pinocchioInterface.getModel();
  pinocchio::Data& data = pinocchioInterface.getData();
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq) * M_PI_2;
    Eigen::VectorXd dq = Eigen::VectorXd::Random(model.nq) * M_PI;
    pinocchio::framesForwardKinematics(model, data, q + dq);

    std::vector<Eigen::Vector3d> threeDofPositions;

    std::vector<size_t> endEffectorIndexes;

    for(const auto& name: threeDofLinks)
    {
      endEffectorIndexes.push_back(model.getFrameId(name));
    }

    for(size_t i = 0; i < 4; ++i)
    {
      threeDofPositions.push_back(data.oMf[endEffectorIndexes[i]].translation());
    }

    Eigen::VectorXd qInverse;
    for (auto _ : state)
      benchmark::DoNotOptimize(kinematicsTest.calculateJointPositions(q, threeDofPositions, 
        qInverse));
}

BENCHMARK(classicCalculateJointPositionsThreeDoFsSmallChange);
BENCHMARK(classicCalculateJointPositionsThreeDoFsBigChange);

BENCHMARK(ADCalculateJointPositionsThreeDoFsSmallChange);
BENCHMARK(ADCalculateJointPositionsThreeDoFsBigChange);

BENCHMARK_MAIN();