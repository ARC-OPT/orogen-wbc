/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcTask.hpp"
#include <wbc/core/Scene.hpp>
#include <wbc/core/RobotModel.hpp>
#include <wbc/core/QPSolver.hpp>
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
    if(!robot_model->configure(_robot_model.get()))
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
    joint_weights = _initial_joint_weights.get();
    if(joint_weights.size() == 0)
        joint_weights.setOnes(robot_model->noOfActuatedJoints());

    LOG_DEBUG("... Created ports");

    compute_constraint_status = _compute_constraint_status.get();
    integrate = _integrate.get();

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

    _joint_weights.readNewest(joint_weights);
    _joint_state.readNewest(joint_state);
    if(joint_state.empty()){
        if(state() != NO_JOINT_STATE)
            state(NO_JOINT_STATE);
        return;
    }
    if(state() != RUNNING)
        state(RUNNING);

    _floating_base_state.readNewest(floating_base_state);
    if(_floating_base_state_deprecated.readNewest(floating_base_state_rbs) == RTT::NewData){
        floating_base_state.pose.position = floating_base_state_rbs.position;
        floating_base_state.pose.orientation = floating_base_state_rbs.orientation;
        floating_base_state.twist.linear = floating_base_state_rbs.velocity;
        floating_base_state.twist.angular = floating_base_state_rbs.angular_velocity;
        floating_base_state.time = floating_base_state_rbs.time;
        floating_base_state.frame_id = floating_base_state_rbs.targetFrame;
    }


    // Update Robot Model
    robot_model->update(joint_state, floating_base_state);

    // Update constraints
    for(const auto& it : constraint_interfaces)
        it.second->update();

    // Update Quadratic program
    hierarchical_qp = wbc_scene->update();

    // Solve
    hierarchical_qp.Wq = joint_weights;
    _current_joint_weights.write(hierarchical_qp.Wq);
    solver_output_joints = wbc_scene->solve(hierarchical_qp);
    if(integrate)
        integrator.integrate(joint_state, solver_output_joints, this->getPeriod());
    _solver_output.write(solver_output_joints);

    // Write debug output
    _full_joint_state.write(robot_model->jointState(robot_model->jointNames()));
    _computation_time.write((base::Time::now() - cur).toSeconds());
    _current_qp.write(hierarchical_qp);
    full_joint_state = robot_model->jointState(robot_model->jointNames());
    if(compute_constraint_status){
        constraints_status = wbc_scene->updateConstraintsStatus();
        for(const auto &c : constraint_interfaces)
            c.second->writeConstraintStatus(constraints_status[c.first]);
    }
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
    solver_output_joints.clear();
    full_joint_state.clear();

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

