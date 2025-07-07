#include <end_effector_kinematics/QuadrupedBaseController.hpp>
#include <end_effector_kinematics/aaa.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    pinocchio::Model model;
    pinocchio::urdf::buildModel("/home/bartek/test/meldog.urdf"
    , model, false);
    pinocchio::Data data(model);
    std::vector<std::string> feet_names{"FLF_joint", "BLF_joint", "FRF_joint", "BRF_joint"};
    auto quadruped_base_controller = std::make_shared<QuadrupedBaseController>(model, data, feet_names);
    rclcpp::spin(quadruped_base_controller);
    rclcpp::shutdown();
    return 0;
}