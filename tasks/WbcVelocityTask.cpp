/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcVelocityTask.hpp"
#include "wbcTypes.hpp"
#include <base/logging.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl_conversions/KDLConversions.hpp>

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
    joint_weights_ = _initial_joint_weights.get();
    compute_debug_ = _compute_debug.get();
    std::vector<std::string> joint_names = _joint_names.get();
    std::string urdf = _urdf.get();
    std::vector<wbc::URDFModel> urdf_models = _urdf_models.get();

    // Configure wbc:
    if(!wbc_.configure(wbc_config, joint_names)){
        LOG_ERROR("Unable to configure WBC");
        return false;
    }
    LOG_DEBUG("... Configured WBC");


    // Load URDF Models and add them to kinematic model:
    if(!urdf.empty()){
        if(!addURDFModel(URDFModel(urdf)))
            return false;
    }
    if(!urdf_models.empty()){
        for(uint i = 0; i < urdf_models.size(); i++){
            if(!addURDFModel(urdf_models[i]))
                return false;
        }
    }

    // Add task frames
    kinematic_model_.addTaskFrames(wbc_.getTaskFrameIDs());

    LOG_DEBUG("... Configured Robot Model");

    // Configure solver
    solver_.setNormMax(_norm_max.get());
    solver_.setEpsilon(_epsilon.get());
    std::vector<int> nc_pp = wbc_.getNumberOfConstraintsPP();
    if(!solver_.configure(nc_pp, wbc_.noOfJoints())){
        LOG_ERROR("Unable to configure wbc solver");
        return false;
    }

    LOG_DEBUG("... Configured Solver");

    //
    // Create Ports
    //

    for(uint i = 0; i < wbc_config.size(); i++)
        constraint_interfaces_.push_back(new ConstraintInterface(wbc_.constraint(wbc_config[i].name), this));

    LOG_DEBUG("... Created ports");

    if(joint_weights_.size() != wbc_.noOfJoints()){
        LOG_ERROR("Number of configured joints is %i, but initial joint weights vector has size %i",  wbc_.noOfJoints(), joint_weights_.size());
        return false;
    }

    equations_.resize(nc_pp.size());
    for(uint prio = 0; prio < nc_pp.size(); prio++)
        equations_[prio].resize(nc_pp[prio], wbc_.noOfJoints());
    solver_output_.resize(wbc_.noOfJoints());
    solver_output_.setZero();
    ctrl_out_.resize(wbc_.noOfJoints());
    ctrl_out_.names = joint_names;
    robot_vel_.resize(wbc_.noOfJoints());
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
    for(uint i = 0; i < constraint_interfaces_.size(); i++)
        constraint_interfaces_[i]->reset();
    stamp_.microseconds = 0;
    return true;
}

void WbcVelocityTask::updateHook(){
    WbcVelocityTaskBase::updateHook();

    base::Time cur = base::Time::now();
    if(!stamp_.isNull())
        _actual_cycle_time.write((cur - stamp_).toSeconds());
    stamp_ = cur;

    if(_joint_state.read(joint_state_) == RTT::NoData){
        if(state() != WAITING_FOR_JOINT_STATE)
            state(WAITING_FOR_JOINT_STATE);
        return;
    }

    for(uint i = 0; i < constraint_interfaces_.size(); i++)
        constraint_interfaces_[i]->update(joint_state_);

    _joint_weights.read(joint_weights_);

    if(state() != RUNNING)
        state(RUNNING);

    //Update Robot Model
    for(uint i = 0; i <  kinematic_model_interfaces.size(); i++)
        kinematic_model_interfaces[i]->update();
    kinematic_model_.updateJoints(joint_state_);

    // Prepare Equation system
    wbc_.prepareEqSystem(kinematic_model_.getTaskFrameMap(), equations_);
    for(uint prio  = 0; prio < equations_.size(); prio++)
        equations_[prio].W_col = joint_weights_;

    // Solve Equation System
    solver_.solve(equations_, solver_output_);
    for(uint i = 0; i < ctrl_out_.size(); i++)
        ctrl_out_[i].speed = solver_output_(i);


    //Compute debug data
    if(compute_debug_)
    {
        for(uint i = 0; i <ctrl_out_.size(); i++)
            robot_vel_(i) = joint_state_.getElementByName(ctrl_out_.names[i]).speed;

        wbc_.evaluateConstraints(solver_output_, robot_vel_);

        for(uint prio = 0; prio < equations_.size(); prio++) // Loop priorities
        {
            damping_[prio] = solver_.getPriorityData(prio).damping_;
            singular_values_[prio] = solver_.getPriorityData(prio).singular_values_;

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

        _inv_condition_number_pp.write(inv_condition_numbers_);
        _damping_pp.write(damping_);
        //_singular_values_pp.write(singular_values_);
        _manipulability_pp.write(manipulability_);
    }

    // Write outputs
    _computation_time.write((base::Time::now() - cur).toSeconds());
    _current_joint_weights.write(joint_weights_);
    wbc_.getConstraintVector(constraints_);
    _constraints.write(constraints_);
    ctrl_out_.time = base::Time::now();
    _ctrl_out.write(ctrl_out_);

    // Write task frames
    const TaskFrameMap& tf_map = kinematic_model_.getTaskFrameMap();
    if(task_frames.size() != tf_map.size())
        task_frames.resize(tf_map.size());

    uint i = 0;
    base::Time t = base::Time::now();
    for(TaskFrameMap::const_iterator it = tf_map.begin(); it != tf_map.end(); it++){
        const TaskFrame& tf = it->second;
        kdl_conversions::KDL2RigidBodyState(tf.pose, task_frames[i]);
        task_frames[i].sourceFrame = tf.tipFrame();
        task_frames[i].targetFrame = kinematic_model_.getRootName();
        task_frames[i].time = t;
        i++;
    }
    _task_frames.write(task_frames);
}


void WbcVelocityTask::stopHook(){

    //Set speed to zero
    for(uint i = 0; i < ctrl_out_.size(); i++)
        ctrl_out_[i].speed = 0.0;
    _ctrl_out.write(ctrl_out_);

    WbcVelocityTaskBase::stopHook();
}

void WbcVelocityTask::cleanupHook()
{
    WbcVelocityTaskBase::cleanupHook();

    // Clear constraint interfaces
    for(uint i = 0; i < constraint_interfaces_.size(); i++)
        delete constraint_interfaces_[i];
    constraint_interfaces_.clear();

    // Clear kinematic model interfaces
    for(uint i = 0; i < kinematic_model_interfaces.size(); i++)
        delete kinematic_model_interfaces[i];
    kinematic_model_interfaces.clear();
}

bool WbcVelocityTask::addURDFModel(wbc::URDFModel const & model){

    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(model.file, tree)){
        LOG_ERROR("Unable to parse urdf file %s into kdl tree", model.file.c_str());
        return false;
    }
    kinematic_model_interfaces.push_back(new KinematicModelInterface(&kinematic_model_, tree.getRootSegment()->first, this));
    return kinematic_model_.addTree(tree, model.initial_pose, model.hook);
}
