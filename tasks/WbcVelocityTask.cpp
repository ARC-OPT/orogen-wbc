/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcVelocityTask.hpp"
#include "wbcTypes.hpp"
#include <base/logging.h>
#include <wbc/WbcVelocity.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <wbc/HierarchicalWDLSSolver.hpp>
#include <wbc/RobotModelKDL.hpp>

using namespace wbc;
using namespace std;

WbcVelocityTask::WbcVelocityTask(std::string const& name)
    : WbcVelocityTaskBase(name){
}

WbcVelocityTask::WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcVelocityTaskBase(name, engine){
}

bool WbcVelocityTask::configureHook(){
    if (! WbcVelocityTaskBase::configureHook())
        return false;

    std::vector<wbc::ConstraintConfig> wbc_config = _wbc_config.get();
    std::vector<std::string> joint_names = _joint_names.get();
    joint_weights_ = _initial_joint_weights.get();

    //
    // Configure WBC Library
    //

    wbc_ = new WbcVelocity();
    if(!wbc_->configure(wbc_config, joint_names, _task_timeout.get())){
        LOG_ERROR("Unable to configure WBC");
        return false;
    }
    LOG_DEBUG("... Configured WBC");

    //
    // Configure Robot model
    //

    // Load URDF Model:
    KDL::Tree full_tree, tree;
    if(!kdl_parser::treeFromFile(_urdf.get(), full_tree))
    {
        LOG_ERROR("Unable to parse KDL Tree from URDF file %s", _urdf.get().c_str());
        return false;
    }

    //Construct tree, if no reduced tree is given, use full tree
    std::vector<wbc::SubChainConfig> reduced_tree = _reduced_tree.get();
    if(reduced_tree.empty())
        tree = full_tree;
    else{
        for(uint i = 0; i < reduced_tree.size(); i++){
            KDL::Chain chain;
            if(!full_tree.getChain(reduced_tree[i].root, reduced_tree[i].tip, chain))
            {
                LOG_ERROR("Could not extract sub chain between %s and %s from KDL tree", reduced_tree[i].root.c_str(), reduced_tree[i].tip.c_str());
                return false;
            }
            //Root of first subchain will be root of whole KDL tree
            if(i == 0)
                tree.addSegment(KDL::Segment(reduced_tree[i].root, KDL::Joint(reduced_tree[i].root,KDL::Joint::None),KDL::Frame::Identity()), "root");
            tree.addChain(chain, reduced_tree[i].root);
        }
    }

    //Create robot model and add task frames
    robot_model_ = new RobotModelKDL(tree);
    if(!robot_model_->addTaskFrames(wbc_->getTaskFrameIDs()))
        return false;
    LOG_DEBUG("... Configured Robot Model");


    //
    // Configure Solver
    //

    solver_ = new HierarchicalWDLSSolver();
    solver_->setNormMax(_norm_max.get());
    solver_->setSVDMethod(_svd_method.get());
    solver_->setEpsilon(_epsilon.get());

    std::vector<int> nc_pp = wbc_->getNumberOfConstraintsPP();
    if(!solver_->configure(nc_pp, wbc_->noOfJoints())){
        LOG_ERROR("Unable to configure wbc solver");
        return false;
    }
    LOG_DEBUG("... Configured Solver");

    //
    // Create Ports
    //

    for(uint i = 0; i < wbc_config.size(); i++)
    {
        ConstraintInterface* ci = new ConstraintInterface(wbc_->constraint(wbc_config[i].name));
        ci->addPortsToTaskContext(this);
        constraint_interface_map_[wbc_config[i].name] = ci;
    }

    LOG_DEBUG("... Created ports");

    if(joint_weights_.size() != wbc_->noOfJoints()){
        LOG_ERROR("Number of configured joints is %i, but initial joint weights vector has size %i",  wbc_->noOfJoints(), joint_weights_.size());
        return false;
    }

    equations_.resize(nc_pp.size());
    for(uint prio = 0; prio < nc_pp.size(); prio++)
        equations_[prio].resize(nc_pp[prio], wbc_->noOfJoints());
    solver_output_.resize(wbc_->noOfJoints());
    solver_output_.setZero();
    ctrl_out_.resize(wbc_->noOfJoints());
    ctrl_out_.names = joint_names;
    robot_vel_.resize(wbc_->noOfJoints());
    robot_vel_.setZero();
    singular_values_.resize(nc_pp.size());
    inv_condition_numbers_.resize(nc_pp.size());
    damping_.resize(nc_pp.size());
    manipulability_.resize(nc_pp.size());

    return true;
}

bool WbcVelocityTask::startHook(){
    if (! WbcVelocityTaskBase::startHook())
        return false;

    //Clear all task references, weights etc. to have to secure initial state
    for(ConstraintInterfaceMap::iterator it = constraint_interface_map_.begin(); it != constraint_interface_map_.end(); it++)
        it->second->reset();
    stamp_.microseconds = 0;
    return true;
}

void WbcVelocityTask::updateHook(){
    WbcVelocityTaskBase::updateHook();

    base::Time cur = base::Time::now();
    if(!stamp_.isNull())
        _actual_cycle_time.write((cur - stamp_).toSeconds());
    stamp_ = cur;

    for(ConstraintInterfaceMap::iterator it = constraint_interface_map_.begin(); it != constraint_interface_map_.end(); it++)
        it->second->update();

    _joint_weights.read(joint_weights_);

    if(_joint_state.read(joint_state_) == RTT::NoData){
        if(state() != WAITING_FOR_JOINT_STATE)
            state(WAITING_FOR_JOINT_STATE);
        return;
    }

    if(state() != RUNNING)
        state(RUNNING);

    //Update Robot Model
    robot_model_->update(joint_state_);
    robot_model_->getTFVector(task_frames_);

    // Prepare Equation system
    wbc_->prepareEqSystem(task_frames_, equations_);
    wbc_->getConstraintVector(constraints_);
    for(uint prio  = 0; prio < equations_.size(); prio++)
        equations_[prio].W_col = joint_weights_;

    // Solve Equation System
    solver_->solve(equations_, (Eigen::VectorXd& )solver_output_);
    for(uint i = 0; i < ctrl_out_.size(); i++)
        ctrl_out_[i].speed = solver_output_(i);


    //Compute debug data
    for(uint prio = 0; prio < equations_.size(); prio++)
    {
        for(uint i = 0; i < constraints_[prio].size(); i++)
        {
            for(uint j = 0; j <ctrl_out_.size(); j++)
                robot_vel_(j) = joint_state_.getElementByName(ctrl_out_.names[j]).speed;

            constraints_[prio][i].y_solution = constraints_[prio][i].A * solver_output_;
            constraints_[prio][i].y = constraints_[prio][i].A * robot_vel_;
            constraints_[prio][i].error_y_solution = constraints_[prio][i].y_ref - constraints_[prio][i].y_solution;
            constraints_[prio][i].error_y = constraints_[prio][i].y_ref - constraints_[prio][i].y;
        }

        damping_[prio] = solver_->getPriorityData(prio).damping_;
        singular_values_[prio] = solver_->getPriorityData(prio).singular_values_;

        //Find min and max singular value. Since some singular values might be zero due to deactivated
        //constraints (zero row weight). Only consider the singular values, which correspond to rows with non-zero weights

        double max_s_val = singular_values_[prio].maxCoeff();
        double min_s_val = base::infinity<double>();
        manipulability_[prio] = 1;
        for(uint i = 0; i < singular_values_[prio].size(); i++)
        {
            if(singular_values_[prio](i) < min_s_val && singular_values_[prio](i) > 1e-5)
            {
                min_s_val = singular_values_[prio](i);
                manipulability_[prio] *= singular_values_[prio](i);
            }
        }
        //If all row weights are zero, inverse condition number should be 0
        if(min_s_val == base::infinity<double>())
            min_s_val = 0;
        if(manipulability_[prio] == 1)
            manipulability_[prio] = 0;

        inv_condition_numbers_[prio] = min_s_val / max_s_val;
    }


    // Write outputs
    _inv_condition_number_pp.write(inv_condition_numbers_);
    _damping_pp.write(damping_);
    _singular_values_pp.write(singular_values_);
    _manipulability_pp.write(manipulability_);
    _computation_time.write((base::Time::now() - cur).toSeconds());
    _current_joint_weights.write(joint_weights_);
    _constraints.write(constraints_);
    _ctrl_out.write(ctrl_out_);
}

void WbcVelocityTask::cleanupHook()
{
    WbcVelocityTaskBase::cleanupHook();

    for(ConstraintInterfaceMap::iterator it = constraint_interface_map_.begin(); it != constraint_interface_map_.end(); it++){
        it->second->removePortsFromTaskContext(this);
        delete it->second;
    }
    constraint_interface_map_.clear();
    delete wbc_;
    delete solver_;
    delete robot_model_;
}
