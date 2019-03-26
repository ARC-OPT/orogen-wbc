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

    joint_weights = _initial_joint_weights.get();
    if(joint_weights.size() == 0)
        joint_weights.setOnes(robot_model->noOfJoints());
    else if(joint_weights.size() != robot_model->noOfJoints()){
        LOG_ERROR("Number of joint weights is %i but number of robot joints is %i", joint_weights.size(), robot_model->noOfJoints());
        return false;
    }

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
    _robot_model_poses.write(robot_model_interface->getModelPoses());

    // Update constraints
    ConstraintInterfaceMap::const_iterator it;
    for(it = constraint_interfaces.begin(); it != constraint_interfaces.end(); it++)
        it->second->update();

    // Update Quadratic program
    wbc_scene->update();

    // Update joint weights
    _joint_weights.readNewest(joint_weights);

    // Write outputs
    _current_joint_weights.write(joint_weights);

    wbc_scene->getHierarchicalQP(hierarchical_qp);
    // Set equal joint weights for each prio. This is more intuitive and easier for the user to configure
    for(QuadraticProgram &qp : hierarchical_qp.prios)
        qp.Wq = joint_weights;
    hierarchical_qp.time = base::Time::now();
    hierarchical_qp.joint_names = robot_model->jointNames();
    _hierarchical_qp.write(hierarchical_qp);

    // Write debug output
    if(_solver_output.readNewest(solver_output) == RTT::NewData)
        _constraints_status.write(wbc_scene->updateConstraintsStatus(solver_output, joint_state));
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

