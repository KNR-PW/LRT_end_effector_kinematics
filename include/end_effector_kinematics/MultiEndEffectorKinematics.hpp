#ifndef _MULTI_END_EFFECTOR_KINEMATICS_HPP_
#define _MULTI_END_EFFECTOR_KINEMATICS_HPP_

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <vector>
#include <unordered_map>
#include <algorithm>


class MultiEndEffectorKinematics
{

    using frame_iterator_t = std::vector<pinocchio::Frame, Eigen::aligned_allocator<pinocchio::Frame>>::iterator;
    private:
    pinocchio::Model& model_;
    pinocchio::Data& data_;

    int end_effector_amount_;
    std::vector<std::string> end_effector_names_;
    std::vector<pinocchio::Model::Index> end_effector_indexes_;
    
    std::vector<pinocchio::Model::Index> end_effector_subtree_indexes_;
    std::vector<int> end_effecotor_subtree_sizes_;
    std::vector<pinocchio::Model::IndexVector> end_effector_subtree_joints_;

    public:

    MultiEndEffectorKinematics(pinocchio::Model& _model, pinocchio::Data& _data, 
    std::vector<std::string> _end_effector_names);

    void computeCartesianConfiguration(const Eigen::VectorXd& _joint_configuration, 
     std::unordered_map<std::string, pinocchio::SE3>& _end_effector_positions) const;

    void computeJointConfiguration(Eigen::VectorXd& _joint_configuration, 
     const std::unordered_map<std::string, pinocchio::SE3>& _end_effector_positions) const;
};

#endif