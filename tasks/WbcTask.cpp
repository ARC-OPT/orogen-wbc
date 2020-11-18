/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcTask.hpp"
#include <wbc/core/WbcScene.hpp>
#include <wbc/core/RobotModel.hpp>
#include "ConstraintInterface.hpp"
#include <base-logging/Logging.hpp>

using namespace wbc;

WbcTask::WbcTask(std::string const& name)
    : WbcTaskBase(name){
}

WbcTask::WbcTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcTaskBase(name, engine){
}

WbcTask::~WbcTask(){
}

bool WbcTask::configureHook(){
    if (! WbcTaskBase::configureHook())
        return false;

    wbc_config = _wbc_config.get();
    if(!robot_model->configure(_robot_models.get()))
            return false;

    LOG_DEBUG("... Configured Robot Model");

    if(!wbc_scene->configure(wbc_config))
        return false;

    LOG_DEBUG("... Configured WBC Scene");

    // Create constraint interfaces. Don't recreate existing interfaces.
    for(ConstraintConfig cfg : wbc_config){
        if(constraint_interfaces.count(cfg.name) == 0)
            constraint_interfaces[cfg.name] = std::make_shared<ConstraintInterface>(wbc_scene->getConstraint(cfg.name), robot_model, this);
        else
            constraint_interfaces[cfg.name]->constraint = wbc_scene->getConstraint(cfg.name);
    }

    // Remove constraint interfaces that are not required anymore
    ConstraintInterfaceMap::const_iterator it;
    for(const auto &it : constraint_interfaces){
        if(!wbc_scene->hasConstraint(it.first))
            constraint_interfaces.erase(it.first);
    }

    floating_base_state = robot_model->floatingBaseState();

    LOG_DEBUG("... Created ports");

    return true;
}

bool WbcTask::startHook(){
    if (! WbcTaskBase::startHook())
        return false;

    //Clear all task references, weights etc. to have to secure initial state
    for(const auto &it : constraint_interfaces)
        it.second->reset();
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

    _floating_base_state.readNewest(floating_base_state);

    // Update Robot Model
    robot_model->update(joint_state, floating_base_state);

    // Update constraints
    for(const auto& it : constraint_interfaces)
        it.second->update();

    // Update Quadratic program
    wbc_scene->update();
    wbc_scene->getHierarchicalQP(hierarchical_qp);    
    _hierarchical_qp.write(hierarchical_qp);

    // Write debug output
    if(_solver_output.readNewest(solver_output) == RTT::NewData){
        full_joint_state = robot_model->jointState(robot_model->jointNames());
        constraints_status = wbc_scene->updateConstraintsStatus(solver_output, full_joint_state);
        for(const auto &c : constraint_interfaces)
            c.second->writeConstraintStatus(constraints_status[c.first]);
    }
    _full_joint_state.write(robot_model->jointState(robot_model->jointNames()));
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

