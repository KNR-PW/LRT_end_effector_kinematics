#include <benchmark/benchmark.h>
#include <multi_end_effector_kinematics/path_management/package_path.h>
#include <multi_end_effector_kinematics/../../test/include/MultiEndEffectorKinematicsTest.hpp>

using namespace multi_end_effector_kinematics;

static void classicCalculateJointDeltasThreeDoFs(benchmark::State& state)
{
  std::string urdfPathName = package_path::getPath();
  urdfPathName += "/test/models/meldog/meldog_no_base_link.urdf";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = "trunk_link";
  modelSettings.threeDofEndEffectorNames = {"RFF_link", "RRF_link", "LFF_link", "LRF_link"};

  InverseSolverSettings solverSettings;
  solverSettings.dampingCoefficient = 1e-6;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;

  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, solverSettings, "NewtonRaphson");

  auto& pinocchioInterface = kinematicsTest.getPinocchioInterface();
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();

  Eigen::VectorXd q(model.nq);
  q << 0.1, -0.2, 0.3, -0.1, 0.2, -0.3, 0.15, -0.25, 0.35, -0.15, 0.25, -0.35;

  Eigen::VectorXd targetQ(model.nq);
  targetQ << 0.2, -0.1, 0.4, -0.2, 0.1, -0.4, 0.25, -0.15, 0.45, -0.25, 0.15, -0.45;

  pinocchio::framesForwardKinematics(model, data, targetQ);

  std::vector<Eigen::Vector3d> targetPositions;
  targetPositions.reserve(modelSettings.threeDofEndEffectorNames.size());

  for(const auto& name : modelSettings.threeDofEndEffectorNames)
  {
    targetPositions.push_back(data.oMf[model.getFrameId(name)].translation());
  }

  Eigen::VectorXd jointDeltas;

  for(auto _ : state)
  {
    const auto status = kinematicsTest.calculateJointDeltas(q, targetPositions, jointDeltas);

    benchmark::DoNotOptimize(status);
    benchmark::DoNotOptimize(jointDeltas);
    benchmark::ClobberMemory();
  }
}

static void classicCalculateJointPositionsThreeDoFsSmallChange(benchmark::State& state)
{
  std::string urdfPathName = package_path::getPath();
  urdfPathName += "/test/models/meldog/meldog_no_base_link.urdf";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = "trunk_link";
  modelSettings.threeDofEndEffectorNames = {"RFF_link", "RRF_link", "LFF_link", "LRF_link"};

  InverseSolverSettings solverSettings;
  solverSettings.dampingCoefficient = 1e-6;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;

  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, solverSettings, "NewtonRaphson");

  auto& pinocchioInterface = kinematicsTest.getPinocchioInterface();
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();

  Eigen::VectorXd q(model.nq);
  q << 0.1, -0.2, 0.3, -0.1, 0.2, -0.3, 0.15, -0.25, 0.35, -0.15, 0.25, -0.35;

  Eigen::VectorXd targetQ(model.nq);
  targetQ << 0.15, -0.15, 0.35, -0.05, 0.25, -0.25, 0.2, -0.2, 0.4, -0.1, 0.3, -0.3;

  pinocchio::framesForwardKinematics(model, data, targetQ);

  std::vector<Eigen::Vector3d> targetPositions;
  targetPositions.reserve(modelSettings.threeDofEndEffectorNames.size());

  for(const auto& name : modelSettings.threeDofEndEffectorNames)
  {
    targetPositions.push_back(data.oMf[model.getFrameId(name)].translation());
  }

  Eigen::VectorXd solvedJointPositions;

  const auto validationStatus = kinematicsTest.calculateJointPositions(q, targetPositions, solvedJointPositions);

  if(!validationStatus.success || validationStatus.flag != TaskReturnFlag::FINISHED)
  {
    state.SkipWithError("SmallChange IK target does not converge.");
    return;
  }

  for(auto _ : state)
  {
    const auto status = kinematicsTest.calculateJointPositions(q, targetPositions, solvedJointPositions);

    benchmark::DoNotOptimize(status);
    benchmark::DoNotOptimize(solvedJointPositions);
    benchmark::ClobberMemory();
  }
}

static void classicCalculateJointPositionsThreeDoFsBigChange(benchmark::State& state)
{
  std::string urdfPathName = package_path::getPath();
  urdfPathName += "/test/models/meldog/meldog_no_base_link.urdf";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = "trunk_link";
  modelSettings.threeDofEndEffectorNames = {"RFF_link", "RRF_link", "LFF_link", "LRF_link"};

  InverseSolverSettings solverSettings;
  solverSettings.dampingCoefficient = 1e-6;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;

  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, solverSettings, "NewtonRaphson");

  auto& pinocchioInterface = kinematicsTest.getPinocchioInterface();
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();

  Eigen::VectorXd q(model.nq);
  q << 0.1, -0.2, 0.3, -0.1, 0.2, -0.3, 0.15, -0.25, 0.35, -0.15, 0.25, -0.35;

  Eigen::VectorXd targetQ(model.nq);
  targetQ << 0.4, -0.5, 0.6, -0.4, 0.5, -0.6, 0.45, -0.55, 0.65, -0.45, 0.55, -0.65;

  pinocchio::framesForwardKinematics(model, data, targetQ);

  std::vector<Eigen::Vector3d> targetPositions;
  targetPositions.reserve(modelSettings.threeDofEndEffectorNames.size());

  for(const auto& name : modelSettings.threeDofEndEffectorNames)
  {
    targetPositions.push_back(data.oMf[model.getFrameId(name)].translation());
  }

  Eigen::VectorXd solvedJointPositions;

  const auto validationStatus = kinematicsTest.calculateJointPositions(q, targetPositions, solvedJointPositions);

  if(!validationStatus.success || validationStatus.flag != TaskReturnFlag::FINISHED)
  {
    state.SkipWithError("BigChange IK target does not converge.");
    return;
  }

  for(auto _ : state)
  {
    const auto status = kinematicsTest.calculateJointPositions(q, targetPositions, solvedJointPositions);

    benchmark::DoNotOptimize(status);
    benchmark::DoNotOptimize(solvedJointPositions);
    benchmark::ClobberMemory();
  }
}

static void ADCalculateJointPositionsThreeDoFsSmallChange(benchmark::State &state) 
{
  std::string urdfPathName = package_path::getPath();
  urdfPathName += "/test/models/meldog/meldog_no_base_link.urdf";

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
  urdfPathName += "/test/models/meldog/meldog_no_base_link.urdf";

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

static void QUIKCalculateJointPositionsThreeDoFsSmallChange(benchmark::State &state) 
{
  std::string urdfPathName = package_path::getPath();
  urdfPathName += "/test/models/meldog/meldog_no_base_link.urdf";

  std::string baseLinkName = "trunk_link";
  std::string rightForwardFeet = "RFF_link";
  std::string leftForwardFeet = "LFF_link";
  std::string rightRearFeet = "RRF_link";
  std::string leftRearFeet = "LRF_link";
  std::vector<std::string> threeDofLinks{rightForwardFeet, rightRearFeet, leftForwardFeet, leftRearFeet};
  std::vector<std::string> sixDofLinks;
  std::string solverName = "QuIK";

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

static void QUIKCalculateJointPositionsThreeDoFsBigChange(benchmark::State &state) 
{
  std::string urdfPathName = package_path::getPath();
  urdfPathName += "/test/models/meldog/meldog_no_base_link.urdf";

  std::string baseLinkName = "trunk_link";
  std::string rightForwardFeet = "RFF_link";
  std::string leftForwardFeet = "LFF_link";
  std::string rightRearFeet = "RRF_link";
  std::string leftRearFeet = "LRF_link";
  std::vector<std::string> threeDofLinks{rightForwardFeet, rightRearFeet, leftForwardFeet, leftRearFeet};
  std::vector<std::string> sixDofLinks;
  std::string solverName = "QuIK";

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

BENCHMARK(classicCalculateJointDeltasThreeDoFs);

BENCHMARK(classicCalculateJointPositionsThreeDoFsSmallChange);
BENCHMARK(classicCalculateJointPositionsThreeDoFsBigChange);

BENCHMARK(ADCalculateJointPositionsThreeDoFsSmallChange);
BENCHMARK(ADCalculateJointPositionsThreeDoFsBigChange);

BENCHMARK(QUIKCalculateJointPositionsThreeDoFsSmallChange);
BENCHMARK(QUIKCalculateJointPositionsThreeDoFsBigChange);

BENCHMARK_MAIN();