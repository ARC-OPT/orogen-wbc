/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcTask.hpp"
#include <wbc/WbcScene.hpp>
#include <wbc/RobotModel.hpp>
#include "ConstraintInterface.hpp"
#include "RobotModelInterface.hpp"
#include <base-logging/Logging.hpp>

using namespace wbc;

WbcTask::WbcTask(std::string const& name)
    : WbcTaskBase(name){
    robot_model_interface = std::make_shared<RobotModelInterface>(this);
    robot_model_interface->configure(_robot_models.get());
}

WbcTask::WbcTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcTaskBase(name, engine){
}

WbcTask::~WbcTask(){
    // Delete port interfaces here, so that the ports are not erased when WBC is (re-)configured
    robot_model_interface.reset();
    for(ConstraintInterfaceMap::const_iterator it = constraint_interfaces.begin(); it != constraint_interfaces.end(); it++)
        it->second->reset();
    constraint_interfaces.clear();
}

bool WbcTask::configureHook(){
    if (! WbcTaskBase::configureHook())
        return false;

    wbc_config = _wbc_config.get();

    if(!robot_model->configure(_robot_models.get(), _joint_names.get(), _base_frame.get()))
            return false;

    LOG_DEBUG("... Configured Robot Model");

    if(!wbc_scene->configure(wbc_config))
        return false;

    LOG_DEBUG("... Configured WBC Scene");

    // Create constraint interfaces. Don't recreate existing interfaces.
    for(uint i = 0; i < wbc_config.size(); i++){
        if(constraint_interfaces.count(wbc_config[i].name) == 0)
            constraint_interfaces[wbc_config[i].name] = std::make_shared<ConstraintInterface>(wbc_scene->getConstraint(wbc_config[i].name), robot_model, this);
        else
            constraint_interfaces[wbc_config[i].name]->constraint = wbc_scene->getConstraint(wbc_config[i].name);
    }

    // Remove constraint interfaces that are not required anymore
    ConstraintInterfaceMap::const_iterator it;
    for(it = constraint_interfaces.begin(); it != constraint_interfaces.end(); it++){
        if(!wbc_scene->hasConstraint(it->first))
            constraint_interfaces.erase(it->first);
    }

    // Configure robot model interface
    robot_model_interface->configure(_robot_models.get());

    LOG_DEBUG("... Created ports");

    return true;
}

bool WbcTask::startHook(){
    if (! WbcTaskBase::startHook())
        return false;

    //Clear all task references, weights etc. to have to secure initial state
    ConstraintInterfaceMap::const_iterator it;
    for(it = constraint_interfaces.begin(); it != constraint_interfaces.end(); it++)
        it->second->reset();
    stamp.microseconds = 0;

    return true;
}

void WbcTask::updateHook(){
    // Compute cycle time
    base::Time cur = base::Time::now();
    if(!stamp.isNull())
        _actual_cycle_time.write((cur - stamp).toSeconds());
    stamp = cur;

    WbcTaskBase::updateHook();

    _joint_state.readNewest(joint_state);
    if(joint_state.empty()){
        if(state() != NO_JOINT_STATE)
            state(NO_JOINT_STATE);
        return;
    }
    if(state() != RUNNING)
        state(RUNNING);

    // Update Robot Model
    robot_model->update(joint_state, robot_model_interface->update());

    // Update constraints
    ConstraintInterfaceMap::const_iterator it;
    for(it = constraint_interfaces.begin(); it != constraint_interfaces.end(); it++)
        it->second->update();

    // UPdate constraints for opt. problem
    wbc_scene->update();
    updateConstraints();

    // Write debug output
    if(_solver_output.readNewest(solver_output) == RTT::NewData){
        wbc_scene->evaluateConstraints(solver_output, joint_state);
        for(it = constraint_interfaces.begin(); it != constraint_interfaces.end(); it++)
            it->second->writeDebug();
    }
    _computation_time.write((base::Time::now() - cur).toSeconds());
}

void WbcTask::errorHook(){
    WbcTaskBase::errorHook();
}

void WbcTask::stopHook(){
    WbcTaskBase::stopHook();
}

void WbcTask::cleanupHook(){
    WbcTaskBase::cleanupHook();
    joint_state.clear();
}

void WbcTask::activateConstraint(const std::string& constraint_name, double activation){
    wbc_scene->getConstraint(constraint_name)->setActivation(activation);
}

void WbcTask::activateConstraints(const std::vector<std::string>& constraint_names, double activation){
    for(auto name : constraint_names)
         wbc_scene->getConstraint(name)->setActivation(activation);
}

void WbcTask::deactivateAllConstraints(){
    for(auto constraint : wbc_config)
        wbc_scene->getConstraint(constraint.name)->setActivation(0);
}

