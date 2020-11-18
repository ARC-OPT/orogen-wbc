/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcVelocityQuadraticCostTask.hpp"
#include <wbc/robot_models/RobotModelKDL.hpp>
#include <wbc/scenes/WbcVelocitySceneQuadraticCost.hpp>
#include <memory>

using namespace wbc;
using namespace std;

WbcVelocityQuadraticCostTask::WbcVelocityQuadraticCostTask(std::string const& name)
    : WbcVelocityQuadraticCostTaskBase(name){
    robot_model = std::make_shared<RobotModelKDL>();
    wbc_scene = std::make_shared<WbcVelocitySceneQuadraticCost>(robot_model);
}

WbcVelocityQuadraticCostTask::~WbcVelocityQuadraticCostTask(){
}

bool WbcVelocityQuadraticCostTask::configureHook(){
    if (! WbcVelocityQuadraticCostTaskBase::configureHook())
        return false;

    std::dynamic_pointer_cast<WbcVelocitySceneQuadraticCost>(wbc_scene)->setDampingThreshold(_min_eval_damping_threshold.get());

    return true;
}

bool WbcVelocityQuadraticCostTask::startHook(){
    if (! WbcVelocityQuadraticCostTaskBase::startHook())
        return false;
    return true;
}

void WbcVelocityQuadraticCostTask::updateHook(){
    WbcVelocityQuadraticCostTaskBase::updateHook();

    _current_damping_factor.write(std::dynamic_pointer_cast<WbcVelocitySceneQuadraticCost>(wbc_scene)->getCurrentDampingFactor());
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
