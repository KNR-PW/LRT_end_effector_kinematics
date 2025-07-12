#include <lrt_inverse_kinematics/InverseKinematicsInfo.hpp>
#include <lrt_inverse_kinematics/InverseKinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

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

  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdfPathName, model);

  pinocchio::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq);

  pinocchio::computeJointJacobians(model, data, q);

  Eigen::MatrixXd jacobianOne = pinocchio::getFrameJacobian(model, data, 10, pinocchio::WORLD);

  Eigen::MatrixXd jacobianTwo(6, model.nv);

  pinocchio::computeFrameJacobian(model, data, q, 10, pinocchio::WORLD, jacobianTwo);

  std::cout << (jacobianOne - jacobianTwo).norm() << std::endl;


}