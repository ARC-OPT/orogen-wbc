/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcVelocityQuadraticCostTask.hpp"
#include <wbc/robot_models/RobotModelHyrodyn.hpp>
#include <wbc/solvers/qpoases/QPOasesSolver.hpp>
#include <wbc/scenes/VelocitySceneQuadraticCost.hpp>

using namespace wbc;

WbcVelocityQuadraticCostTask::WbcVelocityQuadraticCostTask(std::string const& name)
    : WbcVelocityQuadraticCostTaskBase(name){
    robot_model = std::make_shared<RobotModelHyrodyn>();
    solver = std::make_shared<QPOASESSolver>();
    wbc_scene = std::make_shared<VelocitySceneQuadraticCost>(robot_model, solver);
}

WbcVelocityQuadraticCostTask::~WbcVelocityQuadraticCostTask(){
}

bool WbcVelocityQuadraticCostTask::configureHook(){
    if (! WbcVelocityQuadraticCostTaskBase::configureHook())
        return false;

    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(_n_wsr.get());
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(_solver_options.get());

    return true;
}
bool WbcVelocityQuadraticCostTask::startHook(){
    if (! WbcVelocityQuadraticCostTaskBase::startHook())
        return false;
    return true;
}
void WbcVelocityQuadraticCostTask::updateHook(){
    WbcVelocityQuadraticCostTaskBase::updateHook();
}
void WbcVelocityQuadraticCostTask::errorHook(){
    WbcVelocityQuadraticCostTaskBase::errorHook();
}
void WbcVelocityQuadraticCostTask::stopHook(){
    WbcVelocityQuadraticCostTaskBase::stopHook();
}
void WbcVelocityQuadraticCostTask::cleanupHook(){
    WbcVelocityQuadraticCostTaskBase::cleanupHook();
}
