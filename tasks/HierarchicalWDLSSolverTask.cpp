/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "HierarchicalWDLSSolverTask.hpp"

using namespace wbc;

HierarchicalWDLSSolverTask::HierarchicalWDLSSolverTask(std::string const& name)
    : HierarchicalWDLSSolverTaskBase(name)
{
}

HierarchicalWDLSSolverTask::HierarchicalWDLSSolverTask(std::string const& name, RTT::ExecutionEngine* engine)
    : HierarchicalWDLSSolverTaskBase(name, engine)
{
}

HierarchicalWDLSSolverTask::~HierarchicalWDLSSolverTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See HierarchicalWDLSSolverTask.hpp for more detailed
// documentation about them.

bool HierarchicalWDLSSolverTask::configureHook()
{
    if (! HierarchicalWDLSSolverTaskBase::configureHook())
        return false;
    return true;
}
bool HierarchicalWDLSSolverTask::startHook()
{
    if (! HierarchicalWDLSSolverTaskBase::startHook())
        return false;
    return true;
}
void HierarchicalWDLSSolverTask::updateHook()
{
    HierarchicalWDLSSolverTaskBase::updateHook();
}
void HierarchicalWDLSSolverTask::errorHook()
{
    HierarchicalWDLSSolverTaskBase::errorHook();
}
void HierarchicalWDLSSolverTask::stopHook()
{
    HierarchicalWDLSSolverTaskBase::stopHook();
}
void HierarchicalWDLSSolverTask::cleanupHook()
{
    HierarchicalWDLSSolverTaskBase::cleanupHook();
}
