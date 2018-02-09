/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SolverTask.hpp"

using namespace wbc;

SolverTask::SolverTask(std::string const& name)
    : SolverTaskBase(name){
}

SolverTask::SolverTask(std::string const& name, RTT::ExecutionEngine* engine)
    : SolverTaskBase(name, engine){
}

SolverTask::~SolverTask(){
}

bool SolverTask::configureHook(){
    if (! SolverTaskBase::configureHook())
        return false;
    return true;
}

bool SolverTask::startHook(){
    if (! SolverTaskBase::startHook())
        return false;
    return true;
}

void SolverTask::updateHook(){    
    // Compute cycle time
    base::Time cur = base::Time::now();
    if(!stamp.isNull())
        _actual_cycle_time.write((cur - stamp).toSeconds());
    stamp = cur;

    SolverTaskBase::updateHook();

    computeSolverOutput(solver_output);
    if(!solver_output.empty())
        _solver_output.write(solver_output);

    _computation_time.write((base::Time::now() - stamp).toSeconds());
}

void SolverTask::errorHook(){
    SolverTaskBase::errorHook();
}

void SolverTask::stopHook(){
    SolverTaskBase::stopHook();
    _solver_output.write(solver_output);
}

void SolverTask::cleanupHook(){
    SolverTaskBase::cleanupHook();
}
