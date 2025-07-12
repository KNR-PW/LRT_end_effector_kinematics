#include <lrt_inverse_kinematics/InverseKinematicsInfo.hpp>
#include <lrt_inverse_kinematics/InverseKinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

int main(int argc, char** argv)
{
  std::string urdfPathName = "/home/bartek/KNR/kinematics/src/test/models/meldog_base_link.urdf";
  std::string baseLinkName = "trunk_link";
  std::string rightForwardFeet = "RFF_link";
  std::string leftForwardFeet = "LFF_link";
  std::string rightRearFeet = "RRF_link";
  std::string leftRearFeet = "LRF_link";
  std::vector<std::string> threeDofLinks{rightForwardFeet, rightRearFeet};
  std::vector<std::string> sixDofLinks{leftForwardFeet, leftRearFeet};
  lrt_inverse_kinematics::IKSolverInfo solverInfo;
  lrt_inverse_kinematics::InverseKinematics(urdfPathName,
  baseLinkName, threeDofLinks, sixDofLinks, solverInfo, "test");
}