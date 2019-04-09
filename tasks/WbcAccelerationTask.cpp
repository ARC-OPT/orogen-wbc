/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcAccelerationTask.hpp"
#include <wbc/robot_models/KinematicRobotModelKDL.hpp>
#include <wbc/scenes/WbcAccelerationScene.hpp>

using namespace wbc;

WbcAccelerationTask::WbcAccelerationTask(std::string const& name)
    : WbcAccelerationTaskBase(name){
    robot_model = std::make_shared<KinematicRobotModelKDL>();
    wbc_scene = std::make_shared<WbcAccelerationScene>(robot_model);
}

WbcAccelerationTask::WbcAccelerationTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcAccelerationTaskBase(name, engine){
    robot_model = std::make_shared<KinematicRobotModelKDL>();
    wbc_scene = std::make_shared<WbcAccelerationScene>(robot_model);
}

WbcAccelerationTask::~WbcAccelerationTask(){
}

bool WbcAccelerationTask::configureHook(){
    if (! WbcAccelerationTaskBase::configureHook())
        return false;
    return true;
}

bool WbcAccelerationTask::startHook(){
    if (! WbcAccelerationTaskBase::startHook())
        return false;
    return true;
}

void WbcAccelerationTask::updateHook(){
    WbcAccelerationTaskBase::updateHook();
}

void WbcAccelerationTask::errorHook(){
    WbcAccelerationTaskBase::errorHook();
}

void WbcAccelerationTask::stopHook(){
    WbcAccelerationTaskBase::stopHook();
}

void WbcAccelerationTask::cleanupHook(){
    WbcAccelerationTaskBase::cleanupHook();
}
