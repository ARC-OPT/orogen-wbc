/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcVelocityTask.hpp"

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

    wbc_ = new WbcVelocity();
    if(!wbc_->configure(wbc_config, joint_names, _task_timeout.get())){
        LOG_ERROR("Unable to configure WBC");
        return false;
    }

    LOG_DEBUG("... Configured WBC");

    // Create ports
    for(uint i = 0; i < wbc_config.size(); i++)
    {
        ConstraintInterface* ci = new ConstraintInterface(wbc_->constraint(wbc_config[i].name));
        ci->addPortsToTaskContext(this);
        constraint_interface_map_[wbc_config[i].name] = ci;
    }
    
    LOG_DEBUG("... Created ports");

    solver_input_.column_weights = joint_weights_;
    if(joint_weights_.size() != wbc_->noOfJoints()){
        LOG_ERROR("Number of configured joints is %i, but initial joint weights vector has size %i",  wbc_->noOfJoints(), joint_weights_.size());
        return false;
    }
    solver_input_.doResize(wbc_->getNumberOfConstraints(), wbc_->noOfJoints());
    solver_output_.resize(joint_names.size());
    solver_output_.setZero();
    ctrl_out_.resize(joint_names.size());
    ctrl_out_.names = joint_names;
    robot_vel_.resize(joint_names.size());
    robot_vel_.setZero();

    return true;
}

bool WbcVelocityTask::startHook(){
    if (! WbcVelocityTaskBase::startHook())
        return false;

    //Clear all task references, weights etc. to have to secure initial state
    for(ConstraintInterfaceMap::iterator it = constraint_interface_map_.begin(); it != constraint_interface_map_.end(); it++)
        it->second->reset();

    return true;
}

void WbcVelocityTask::updateHook(){
    WbcVelocityTaskBase::updateHook();

    base::Time start = base::Time::now();

    // Read input ports
    if(_task_frames.read(task_frames_) == RTT::NoData){
        LOG_DEBUG("No Data on task frame port");
        return;
    }
    for(ConstraintInterfaceMap::iterator it = constraint_interface_map_.begin(); it != constraint_interface_map_.end(); it++)
        it->second->update();
    _joint_weights.read(solver_input_.column_weights);
    if(_joint_state.read(joint_state_) != RTT::NoData)
    {
        for(uint i = 0; i < ctrl_out_.size(); i++){
            uint idx;
            try{
                idx = joint_state_.mapNameToIndex(ctrl_out_.names[i]);
            }
            catch(std::exception e){
                continue;
            }
            robot_vel_(i) = joint_state_[idx].speed;
        }
    }
    if(_solver_output.read(solver_output_) == RTT::NewData)
        handleSolverOutput();

    // Prepare Equation system
    wbc_->prepareEqSystem(task_frames_, constraints_);
    prepareSolverInput();

    computeConstraintSatisfaction();

    // Write outputs
    _computation_time.write((base::Time::now() - start).toSeconds());
    _solver_input.write(solver_input_);
    _constraints.write(constraints_);
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
}
