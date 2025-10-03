#include <gtest/gtest.h>
#include <multi_end_effector_kinematics/solvers/gradient_based/NewtonRaphsonSolver.hpp>
#include <multi_end_effector_kinematics/../../test/include/MultiEndEffectorKinematicsTest.hpp>
#include <multi_end_effector_kinematics/path_management/package_path.h>

#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Geometry>

using namespace multi_end_effector_kinematics;


static constexpr ocs2::scalar_t tolerance = 1e-3;
static constexpr size_t numTests = 100;

TEST(MultEndEffectorKinematicsTest, Constructor)
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
  EXPECT_NO_THROW(MultiEndEffectorKinematics(urdfPathName, modelSettings, 
    solverSettings, solverName));

  solverName = "aaa";

  EXPECT_THROW(MultiEndEffectorKinematics(urdfPathName, modelSettings, 
    solverSettings, solverName), std::invalid_argument);

  urdfPathName = "aaaa";
  solverName = "NewtonRaphson";

  EXPECT_THROW(MultiEndEffectorKinematics(urdfPathName, modelSettings, 
    solverSettings, solverName), std::invalid_argument);

  urdfPathName = package_path::getPath();
  urdfPathName += "/../install/multi_end_effector_kinematics/share/multi_end_effector_kinematics/models/meldog/meldog_no_base_link.urdf";
  
  modelSettings.threeDofEndEffectorNames[0] = "aaaa";
  EXPECT_THROW(MultiEndEffectorKinematics(urdfPathName, modelSettings, 
    solverSettings, solverName), std::invalid_argument);
  
}


TEST(MultEndEffectorKinematicsTest, calculateJointPositionsThreeDoF)
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

  std::vector<size_t> endEffectorIndexes;

  for(const auto& name: threeDofLinks)
  {
    endEffectorIndexes.push_back(model.getFrameId(name));
  }

  for(int i = 0; i < numTests; ++i)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq) * M_PI_2;;
    Eigen::VectorXd dq = Eigen::VectorXd::Random(model.nq) / 10;

    pinocchio::framesForwardKinematics(model, data, q + dq);

    std::vector<Eigen::Vector3d> threeDofPositions;

    for(size_t i = 0; i < 4; ++i)
    {
      threeDofPositions.push_back(data.oMf[endEffectorIndexes[i]].translation());
    }

    Eigen::VectorXd qInverse;
    const auto result = kinematicsTest.calculateJointPositions(q, threeDofPositions, qInverse);

    pinocchio::framesForwardKinematics(model, data, qInverse);
    for(size_t i = 0; i < 4; ++i)
    {
      EXPECT_TRUE(threeDofPositions[i].isApprox(data.oMf[endEffectorIndexes[i]].translation(), tolerance));
    }
    EXPECT_TRUE(result.success == true);
    EXPECT_TRUE(result.flag == TaskReturnFlag::FINISHED);
  }
}

TEST(MultEndEffectorKinematicsTest, calculateJointVelocitiesThreeDoF)
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
  solverSettings.dampingCoefficient = 1e-3;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;
  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface pinocchioInterface = kinematicsTest.getPinocchioInterface();

  const pinocchio::Model& model = pinocchioInterface.getModel();
  pinocchio::Data& data = pinocchioInterface.getData();

  std::vector<size_t> endEffectorIndexes;

  for(const auto& name: threeDofLinks)
  {
    endEffectorIndexes.push_back(model.getFrameId(name));
  }

  for(int i = 0; i < numTests; ++i)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq) * M_PI_2;
    Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv) * M_PI;

    std::vector<Eigen::Vector3d> threeDofVelocities;
    pinocchio::forwardKinematics(model, data, q, v);
    for(size_t i = 0; i < 4; ++i)
    {
      threeDofVelocities.push_back(pinocchio::getFrameVelocity(model, data, 
        endEffectorIndexes[i], pinocchio::LOCAL_WORLD_ALIGNED).linear());
    }

    Eigen::VectorXd vInverse;
    const auto result = kinematicsTest.calculateJointVelocities(q, threeDofVelocities, vInverse);

    pinocchio::forwardKinematics(model, data, q, vInverse);
    for(size_t i = 0; i < 4; ++i)
    {
      const Eigen::Vector3d newVelocity = pinocchio::getFrameVelocity(model, data, 
        endEffectorIndexes[i], pinocchio::LOCAL_WORLD_ALIGNED).linear();
      EXPECT_TRUE(threeDofVelocities[i].isApprox(newVelocity, tolerance));
    }

    EXPECT_TRUE(result.success == true);
    EXPECT_TRUE(result.flag == TaskReturnFlag::FINISHED);
  }
}

TEST(MultEndEffectorKinematicsTest, calculateEndEffectorVelocitiesThreeDoF)
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
  solverSettings.dampingCoefficient = 1e-3;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;
  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface pinocchioInterface = kinematicsTest.getPinocchioInterface();

  const pinocchio::Model& model = pinocchioInterface.getModel();
  pinocchio::Data& data = pinocchioInterface.getData();

  std::vector<size_t> endEffectorIndexes;

  for(const auto& name: threeDofLinks)
  {
    endEffectorIndexes.push_back(model.getFrameId(name));
  }

  for(int i = 0; i < numTests; ++i)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq) * M_PI_2;
    Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv) * M_PI;

    std::vector<Eigen::Vector3d> threeDofVelocities;
    pinocchio::forwardKinematics(model, data, q, v);
    for(size_t i = 0; i < 4; ++i)
    {
      threeDofVelocities.push_back(pinocchio::getFrameVelocity(model, data, 
        endEffectorIndexes[i], pinocchio::LOCAL_WORLD_ALIGNED).linear());
    }

    std::vector<Eigen::Vector3d> newVelocities(4);
    const auto result = kinematicsTest.calculateEndEffectorVelocities(q, v, newVelocities);

    for(size_t i = 0; i < 4; ++i)
    {
      EXPECT_TRUE(threeDofVelocities[i].isApprox(newVelocities[i], tolerance));
    }

    EXPECT_TRUE(result.success == true);
    EXPECT_TRUE(result.flag == TaskReturnFlag::FINISHED);
  }
}

TEST(MultEndEffectorKinematicsTest, calculateEndEffectorPosesThreeDoF)
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
  solverSettings.dampingCoefficient = 1e-3;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;
  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface pinocchioInterface = kinematicsTest.getPinocchioInterface();

  const pinocchio::Model& model = pinocchioInterface.getModel();
  pinocchio::Data& data = pinocchioInterface.getData();

  std::vector<size_t> endEffectorIndexes;

  for(const auto& name: threeDofLinks)
  {
    endEffectorIndexes.push_back(model.getFrameId(name));
  }

  for(int i = 0; i < numTests; ++i)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq) * M_PI_2;

    std::vector<Eigen::Vector3d> threeDofPositions;
    pinocchio::framesForwardKinematics(model, data, q);
    for(size_t i = 0; i < 4; ++i)
    {
      threeDofPositions.push_back(data.oMf[endEffectorIndexes[i]].translation());
    }

    std::vector<Eigen::Vector3d> newPositions(4);
    const auto result = kinematicsTest.calculateEndEffectorPoses(q, newPositions);

    for(size_t i = 0; i < 4; ++i)
    {
      EXPECT_TRUE(threeDofPositions[i].isApprox(newPositions[i], tolerance));
    }

    EXPECT_TRUE(result.success == true);
    EXPECT_TRUE(result.flag == TaskReturnFlag::FINISHED);
  }
}

TEST(MultEndEffectorKinematicsTest, calculateJointPositionsSixDoF)
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
  solverSettings.dampingCoefficient = 1e-6;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;
  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface pinocchioInterface = kinematicsTest.getPinocchioInterface();

  const pinocchio::Model& model = pinocchioInterface.getModel();
  pinocchio::Data& data = pinocchioInterface.getData();

  const size_t endEffectorIndex = model.getFrameId("tool0");

  for(int i = 0; i < numTests; ++i)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq) * M_PI_2;
    Eigen::VectorXd dq = Eigen::VectorXd::Random(model.nq) / 10;

    pinocchio::framesForwardKinematics(model, data, q + dq);

    std::vector<pinocchio::SE3> targetTransforms{data.oMf[endEffectorIndex]};

    Eigen::VectorXd qInverse;
    const auto result = kinematicsTest.calculateJointPositions(q, targetTransforms, qInverse);

    pinocchio::framesForwardKinematics(model, data, qInverse);
    EXPECT_TRUE(targetTransforms[0].isApprox(data.oMf[endEffectorIndex], tolerance));

    EXPECT_TRUE(result.success == true);
    EXPECT_TRUE(result.flag == TaskReturnFlag::FINISHED);
  }
}

TEST(MultEndEffectorKinematicsTest, calculateJointVelocitiesSixDoF)
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
  solverSettings.dampingCoefficient = 1e-6;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;
  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface pinocchioInterface = kinematicsTest.getPinocchioInterface();

  const pinocchio::Model& model = pinocchioInterface.getModel();
  pinocchio::Data& data = pinocchioInterface.getData();

  const size_t endEffectorIndex = model.getFrameId("tool0");

  for(int i = 0; i < numTests; ++i)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq) * M_PI_2;
    Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);

    std::vector<Eigen::Vector3d> threeDofVelocities;
    pinocchio::forwardKinematics(model, data, q, v);

    std::vector<pinocchio::Motion> trueVelocity{pinocchio::getFrameVelocity(model, data, 
      endEffectorIndex, pinocchio::LOCAL_WORLD_ALIGNED)};
   
    Eigen::VectorXd vInverse;
    const auto result = kinematicsTest.calculateJointVelocities(q, trueVelocity, vInverse);

    pinocchio::forwardKinematics(model, data, q, vInverse);
    const pinocchio::Motion newVelocity = pinocchio::getFrameVelocity(model, data, 
        endEffectorIndex, pinocchio::LOCAL_WORLD_ALIGNED);
    EXPECT_TRUE(trueVelocity[0].toVector().isApprox(newVelocity.toVector(), tolerance));

    EXPECT_TRUE(result.success == true);
    EXPECT_TRUE(result.flag == TaskReturnFlag::FINISHED);
  }
}

TEST(MultEndEffectorKinematicsTest, calculateEndEffectorPosesSixDoF)
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
  solverSettings.dampingCoefficient = 1e-6;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;
  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface pinocchioInterface = kinematicsTest.getPinocchioInterface();

  const pinocchio::Model& model = pinocchioInterface.getModel();
  pinocchio::Data& data = pinocchioInterface.getData();

  const size_t endEffectorIndex = model.getFrameId("tool0");

  for(int i = 0; i < numTests; ++i)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq) * M_PI_2;

    pinocchio::framesForwardKinematics(model, data, q);

    std::vector<pinocchio::SE3> trueTransforms{data.oMf[endEffectorIndex]};

    std::vector<pinocchio::SE3> transforms(1);

    const auto result = kinematicsTest.calculateEndEffectorPoses(q, transforms);

    EXPECT_TRUE(trueTransforms[0].translation().isApprox(transforms[0].translation(), tolerance));
    EXPECT_TRUE(trueTransforms[0].rotation().isApprox(transforms[0].rotation(), tolerance));

    EXPECT_TRUE(result.success == true);
    EXPECT_TRUE(result.flag == TaskReturnFlag::FINISHED);
  }
}

TEST(MultEndEffectorKinematicsTest, calculateEndEffectorVelocitiesSixDoF)
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
  solverSettings.dampingCoefficient = 1e-6;
  solverSettings.stepCoefficient = 0.8;
  solverSettings.tolerance = 1e-5;
  solverSettings.maxIterations = 1000;
  MultiEndEffectorKinematicsTest kinematicsTest(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface pinocchioInterface = kinematicsTest.getPinocchioInterface();

  const pinocchio::Model& model = pinocchioInterface.getModel();
  pinocchio::Data& data = pinocchioInterface.getData();

  const size_t endEffectorIndex = model.getFrameId("tool0");

  for(int i = 0; i < numTests; ++i)
  {
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq) * M_PI_2;
    Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);

    pinocchio::forwardKinematics(model, data, q, v);

    std::vector<pinocchio::Motion> trueVelocity{pinocchio::getFrameVelocity(model, data, 
      endEffectorIndex, pinocchio::LOCAL_WORLD_ALIGNED)};

    std::vector<pinocchio::Motion> velocity(1);

    const auto result = kinematicsTest.calculateEndEffectorVelocities(q, v, velocity);

    EXPECT_TRUE(trueVelocity[0].toVector().isApprox(velocity[0].toVector(), tolerance));
    
    EXPECT_TRUE(result.success == true);
    EXPECT_TRUE(result.flag == TaskReturnFlag::FINISHED);
  }
}