/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcTask.hpp"
#include <wbc/WbcScene.hpp>
#include <wbc/RobotModel.hpp>
#include <wbc/Solver.hpp>
#include "ConstraintInterface.hpp"
#include "RobotModelInterface.hpp"
#include <base-logging/Logging.hpp>

using namespace wbc;

WbcTask::WbcTask(std::string const& name)
    : WbcTaskBase(name)
{
}

WbcTask::WbcTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcTaskBase(name, engine)
{
}

WbcTask::~WbcTask()
{
}

bool WbcTask::configureHook()
{
    if (! WbcTaskBase::configureHook())
        return false;

    std::vector<ConstraintConfig> wbc_config = _wbc_config.get();

    // Create constraint interfaces. Don't recreate existing interfaces.
    for(uint i = 0; i < wbc_config.size(); i++)
        if(constraint_interfaces.count(wbc_config[i].name) == 0)
            constraint_interfaces[wbc_config[i].name] = new ConstraintInterface(wbc_scene->getConstraint(wbc_config[i].name), robot_model, this);

    // Remove constraint interfaces that are not required anymore
    ConstraintInterfaceMap::const_iterator it;
    for(it = constraint_interfaces.begin(); it != constraint_interfaces.end(); it++){
        if(!wbc_scene->hasConstraint(it->first))
            constraint_interfaces.erase(it->first);
    }

    robot_model_interface->configure(_robot_models.get());

    ctrl_out.resize(robot_model->jointNames().size());
    ctrl_out.names = robot_model->jointNames();

    return true;
}

bool WbcTask::startHook()
{
    if (! WbcTaskBase::startHook())
        return false;

    //Clear all task references, weights etc. to have to secure initial state
    ConstraintInterfaceMap::const_iterator it;
    for(it = constraint_interfaces.begin(); it != constraint_interfaces.end(); it++)
        it->second->reset();
    stamp.microseconds = 0;

    return true;
}

void WbcTask::updateHook()
{
    WbcTaskBase::updateHook();

    // Compute cycle time
    base::Time cur = base::Time::now();
    if(!stamp.isNull())
        _actual_cycle_time.write((cur - stamp).toSeconds());
    stamp = cur;

    // Read joint status
    if(_joint_state.readNewest(joint_state) == RTT::NoData){
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

    // Solve opt. problem
    wbc_scene->solve(ctrl_out);

    _ctrl_out.write(ctrl_out);
    _constraints.write(wbc_scene->getConstraints());

    // Write computation time for one cycle
    _computation_time.write((base::Time::now() - cur).toSeconds());
}

void WbcTask::errorHook()
{
    WbcTaskBase::errorHook();
}
void WbcTask::stopHook()
{
    WbcTaskBase::stopHook();
}
void WbcTask::cleanupHook()
{
    WbcTaskBase::cleanupHook();

    ctrl_out.clear();

    // clear interfaces
    delete robot_model_interface;
    for(ConstraintInterfaceMap::const_iterator it = constraint_interfaces.begin(); it != constraint_interfaces.end(); it++)
        delete it->second;
    constraint_interfaces.clear();
}
