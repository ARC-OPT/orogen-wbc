/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcVelocityTask.hpp"
#include "wbcTypes.hpp"
#include <base/logging.h>
#include <wbc/WbcVelocity.hpp>

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

    // Configure WBC library
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

    if(joint_weights_.size() != wbc_->noOfJoints()){
        LOG_ERROR("Number of configured joints is %i, but initial joint weights vector has size %i",  wbc_->noOfJoints(), joint_weights_.size());
        return false;
    }

    std::vector<uint> nc_pp = wbc_->getNumberOfConstraintsPP();
    equations_.resize(nc_pp.size());
    for(uint prio = 0; prio < nc_pp.size(); prio++)
        equations_[prio].resize(nc_pp[prio], wbc_->noOfJoints());
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

    if(_task_frames.read(task_frames_) == RTT::NoData){
        if(state() == RUNNING)
            state(WAITING_FOR_TASK_FRAMES);
        return;
    }
    if(state() == WAITING_FOR_TASK_FRAMES)
        state(RUNNING);

    for(ConstraintInterfaceMap::iterator it = constraint_interface_map_.begin(); it != constraint_interface_map_.end(); it++)
        it->second->update();

    _joint_weights.read(joint_weights_);

    if(_joint_state.read(joint_state_) != RTT::NoData)
    {
        for(uint i = 0; i < ctrl_out_.size(); i++){
            try{
                robot_vel_(i) = joint_state_.getElementByName(ctrl_out_.names[i]).speed;
            }
            catch(std::exception e){
                LOG_ERROR("WBC expects element %s, but this element is not available in joint state vector", ctrl_out_.names[i].c_str());
                throw std::invalid_argument("Invalid joint state vector");
            }
        }
    }

    if(_solver_output.read(solver_output_) == RTT::NewData)
    {
        if(solver_output_.size() != wbc_->noOfJoints())
        {
            LOG_ERROR("Solver output size should be %i but is %i", wbc_->noOfJoints(), solver_output_.size());
            throw std::invalid_argument("Invalid solver output");
        }
        for(uint i = 0; i < ctrl_out_.size(); i++)
            ctrl_out_[i].speed = solver_output_(i);
        _ctrl_out.write(ctrl_out_);
    }

    // Prepare Equation system
    wbc_->prepareEqSystem(task_frames_, equations_);
    wbc_->getConstraintVector(constraints_);

    for(uint prio  = 0; prio < equations_.size(); prio++)
    {
        equations_[prio].W_col = joint_weights_;
        for(uint i = 0; i < constraints_[prio].size(); i++)
        {
            constraints_[prio][i].y_solution = constraints_[prio][i].A * solver_output_;
            constraints_[prio][i].y = constraints_[prio][i].A * robot_vel_;
        }
    }

    // Write outputs
    _computation_time.write((base::Time::now() - start).toSeconds());
    _linear_eqn_pp.write(equations_);
    _constraints.write(constraints_);
    _current_joint_weights.write(joint_weights_);
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
