/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "HierarchicalLSSolverTask.hpp"

using namespace wbc;

HierarchicalLSSolverTask::HierarchicalLSSolverTask(std::string const& name)
    : HierarchicalLSSolverTaskBase(name){
}

HierarchicalLSSolverTask::HierarchicalLSSolverTask(std::string const& name, RTT::ExecutionEngine* engine)
    : HierarchicalLSSolverTaskBase(name, engine){
}

HierarchicalLSSolverTask::~HierarchicalLSSolverTask(){
}

bool HierarchicalLSSolverTask::configureHook(){
    if (! HierarchicalLSSolverTaskBase::configureHook())
        return false;

    solver = std::make_shared<HierarchicalLSSolver>();
    solver->setMaxSolverOutputNorm(_norm_max.get());
    solver->setMinEigenvalue(_epsilon.get());
    if(_max_solver_output.get().size() > 0)
        solver->setMaxSolverOutput(_max_solver_output.get());

    return true;
}

bool HierarchicalLSSolverTask::startHook(){
    if (! HierarchicalLSSolverTaskBase::startHook())
        return false;
    return true;
}

void HierarchicalLSSolverTask::updateHook(){
    HierarchicalLSSolverTaskBase::updateHook();

}
void HierarchicalLSSolverTask::errorHook(){
    HierarchicalLSSolverTaskBase::errorHook();
}

void HierarchicalLSSolverTask::stopHook(){
    //Set speed to zero
    for(uint i = 0; i < solver_output.size(); i++)
        solver_output[i].speed = 0.0;
    HierarchicalLSSolverTaskBase::stopHook();
}

void HierarchicalLSSolverTask::cleanupHook(){
    HierarchicalLSSolverTaskBase::cleanupHook();
    solver.reset();
}

void HierarchicalLSSolverTask::computeSolverOutput(base::commands::Joints& solver_output){

    if(_constraints_prio.readNewest(constraints_prio) == RTT::NewData){
        solver->solve(constraints_prio.constraints, solver_output_raw);

        solver_output.resize(constraints_prio.joint_names.size());
        solver_output.names = constraints_prio.joint_names;
        for(size_t i = 0; i < solver_output.size(); i++)
            solver_output[i].speed = solver_output_raw(i);
    }
}
