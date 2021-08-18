/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcVelocityTask.hpp"
#include <wbc/robot_models/RobotModelKDL.hpp>
#include <wbc/scenes/VelocityScene.hpp>
#include <wbc/solvers/hls/HierarchicalLSSolver.hpp>
#include "ConstraintInterface.hpp"

using namespace wbc;
using namespace std;

WbcVelocityTask::WbcVelocityTask(std::string const& name)
    : WbcVelocityTaskBase(name){
    robot_model = std::make_shared<RobotModelKDL>();
    solver = std::make_shared<HierarchicalLSSolver>();
    wbc_scene = std::make_shared<VelocityScene>(robot_model, solver);
}

WbcVelocityTask::WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcVelocityTaskBase(name, engine){
    robot_model = std::make_shared<RobotModelKDL>();
    solver = std::make_shared<HierarchicalLSSolver>();
    wbc_scene = std::make_shared<VelocityScene>(robot_model, solver);
}

bool WbcVelocityTask::configureHook(){
    if (! WbcVelocityTaskBase::configureHook())
        return false;

    std::dynamic_pointer_cast<HierarchicalLSSolver>(solver)->setMaxSolverOutputNorm(_norm_max.get());
    std::dynamic_pointer_cast<HierarchicalLSSolver>(solver)->setMinEigenvalue(_epsilon.get());
    compute_id = _compute_id.get();

    return true;
}

bool WbcVelocityTask::startHook(){
    if (! WbcVelocityTaskBase::startHook())
        return false;
    return true;
}

void WbcVelocityTask::updateHook(){
    WbcVelocityTaskBase::updateHook();
}

void WbcVelocityTask::errorHook(){
    WbcVelocityTaskBase::errorHook();
}

void WbcVelocityTask::stopHook(){
    WbcVelocityTaskBase::stopHook();
}

void WbcVelocityTask::cleanupHook(){
    WbcVelocityTaskBase::cleanupHook();
}
