#include <end_effector_kinematics/MultiEndEffectorKinematics.hpp>


MultiEndEffectorKinematics::MultiEndEffectorKinematics(pinocchio::Model& _model, pinocchio::Data& _data, 
    std::vector<std::string> _end_effector_names): model_{_model}, data_{_data}, end_effector_names_{_end_effector_names}
{
    end_effector_amount_ = end_effector_names_.size();
    for(int i = 0; i < end_effector_amount_; i++)
    {
        int frame_index = model_.getFrameId(end_effector_names_[i]);

        int joint_index = model_.frames[frame_index].parentJoint;
        int last_joint_index = joint_index;
        while(joint_index != 0)
        {
            last_joint_index = joint_index;
            joint_index = model_.parents[joint_index];
        }
        end_effector_indexes_.push_back(frame_index);
        end_effector_subtree_indexes_.push_back(last_joint_index);
        end_effecotor_subtree_sizes_.push_back(model_.subtrees[last_joint_index].size());

        // Joints starts from 1 not 0! (0 is 'universe')
        auto end_effector_subtree = model_.subtrees[last_joint_index];
        for(int j = 0; j < end_effector_subtree.size(); ++j)
        {
            end_effector_subtree[j] -= 1;
        }
        end_effector_subtree_joints_.push_back(end_effector_subtree);
        std::cout << "Nazwa Frame'u: " << end_effector_names_[i] << std::endl << "Index Frame'u: " << end_effector_indexes_[i] << std::endl;
        std::cout << "Index i nazwa Jointa poczatku drzewa: " << last_joint_index << ", " << model_.names[last_joint_index] << std::endl << "Rozmiar drzewa: " << model_.subtrees[last_joint_index].size() << std::endl;
        std::cout << "Jointy: " << std::endl;
        for(int j = 0; j < end_effector_subtree_joints_[i].size(); ++j)
        {
            std::cout << "Index: " << end_effector_subtree_joints_[i][j] << " Nazwa: " << model_.names[end_effector_subtree_joints_[i][j]] << std::endl;
        }
    }
}

void MultiEndEffectorKinematics::computeCartesianConfiguration(const Eigen::VectorXd& _joint_configuration,
 std::unordered_map<std::string, pinocchio::SE3>& _end_effector_positions) const
{
    pinocchio::framesForwardKinematics(model_, data_, _joint_configuration);
    pinocchio::updateFramePlacements(model_, data_);
    for(int i = 0; i < end_effector_amount_; i++)
    {
        if(_end_effector_positions.find(end_effector_names_[i]) == _end_effector_positions.end())
        {
            _end_effector_positions.emplace(end_effector_names_[i], data_.oMf[end_effector_indexes_[i]]);
        }
        else
        {
           _end_effector_positions.at(end_effector_names_[i]) = data_.oMf[end_effector_indexes_[i]];
        }
    }
        
}

void MultiEndEffectorKinematics::computeJointConfiguration(Eigen::VectorXd& _joint_configuration, 
 const std::unordered_map<std::string, pinocchio::SE3>& _end_effector_positions) const
{   
    const int max_iterations = 100;
    const double max_error = 1e-3;
    const double dt = 1.0;
    const double damping = 0.05;

    pinocchio::SE3 goal_position;
    pinocchio::SE3 actual_position;
    pinocchio::Data::Matrix6x full_order_jacobian(6, model_.njoints - 1);
    pinocchio::Data::Matrix3x position_jacobian;
    pinocchio::Data::Matrix3x JJt;
    Eigen::VectorXd joint_velocities(model_.njoints-1);

    Eigen::Vector3d error;
    std::vector<Eigen::Vector3d> errors;

    std::vector<int> active_end_effectors;
    
    pinocchio::forwardKinematics(model_, data_, _joint_configuration);
    pinocchio::updateFramePlacements(model_, data_);

    for (int i = 0; i < end_effector_amount_; i++)
    {
        goal_position = _end_effector_positions.at(end_effector_names_[i]);
        actual_position = data_.oMf[end_effector_indexes_[i]];
        error = goal_position.translation() - actual_position.translation();
        if(error.norm() > max_error)
        {
            active_end_effectors.push_back(i);
            errors.push_back(error);
        }
    }
    
    if(active_end_effectors.size() == 0)
    {
        /* Already every end effector is in desired place, return */
       return;
    }

    for(int i = 0; i < max_iterations; i++)
    {
        std::vector<int> solved_end_effectors_indexes;
        pinocchio::forwardKinematics(model_, data_, _joint_configuration);
        pinocchio::updateFramePlacements(model_, data_);
        for(int j = 0; j < active_end_effectors.size(); j++)
        {
            int active_end_effector = active_end_effectors[j];
            goal_position = _end_effector_positions.at(end_effector_names_[active_end_effector]);
            actual_position = data_.oMf[end_effector_indexes_[active_end_effector]];
            error = actual_position.actInv(goal_position).translation();
            if(error.norm() < max_error)
            {
                solved_end_effectors_indexes.push_back(j);
            }
            else
            {
                errors[j] = error;
            }
        }
        
        if(solved_end_effectors_indexes.size() > 0)
        {
            for(std::vector<int>::reverse_iterator solved_end_effector_indexes_iter = solved_end_effectors_indexes.rbegin();
            solved_end_effector_indexes_iter != solved_end_effectors_indexes.rend(); ++solved_end_effector_indexes_iter)
            {
                errors.erase(errors.begin() + *solved_end_effector_indexes_iter);
                active_end_effectors.erase(active_end_effectors.begin() + *solved_end_effector_indexes_iter);
            }
        }
        
        if(active_end_effectors.size() == 0) 
        {
            /* Every end effector is in desired place, return */
            return;
        }
        joint_velocities.setZero();
        
        for(int j = 0; j < active_end_effectors.size(); j++)
        {   
            
            int active_end_effector = active_end_effectors[j];
            error = errors[j];
            full_order_jacobian.setZero();

            pinocchio::computeFrameJacobian(model_, data_, _joint_configuration, end_effector_indexes_[active_end_effector],
             pinocchio::LOCAL, full_order_jacobian);
            position_jacobian = full_order_jacobian(Eigen::seq(0,2), end_effector_subtree_joints_[active_end_effector]);
            JJt.noalias() = position_jacobian * position_jacobian.transpose();
            JJt.diagonal().array() += damping;
            joint_velocities(end_effector_subtree_joints_[active_end_effector]).noalias() = position_jacobian.transpose() * JJt.ldlt().solve(error);
        }
        _joint_configuration = pinocchio::integrate(model_, _joint_configuration, joint_velocities*dt);
    }
}