#include <lrt_inverse_kinematics/InverseKinematicsInfo.hpp>
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
  lrt_inverse_kinematics::IKModelInfo IkModel(urdfPathName,
  baseLinkName,
  threeDofLinks,
  sixDofLinks);

  const auto& pinocchioInterface = IkModel.pinocchioInterface_;

  const auto& model = pinocchioInterface->getModel();

  for(int i = 0; i < model.njoints; ++i)
  {
    std::cout << "Joint name: " << model.names[i] << std::endl;
  }

  for(int i = 0; i < model.frames.size(); ++i)
  {
    if(model.frames[i].name == "LFH_link") break;
    std::cout << model.frames[i] << std::endl;
  }

}