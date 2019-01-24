/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcVelocityTask.hpp"
#include <wbc/KinematicRobotModelKDL.hpp>
#include <wbc/WbcVelocityScene.hpp>

using namespace wbc;
using namespace std;

WbcVelocityTask::WbcVelocityTask(std::string const& name)
    : WbcVelocityTaskBase(name){
    robot_model = std::make_shared<KinematicRobotModelKDL>();
    wbc_scene = std::make_shared<WbcVelocityScene>(robot_model);

}

WbcVelocityTask::WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcVelocityTaskBase(name, engine){
    robot_model = std::make_shared<KinematicRobotModelKDL>();
    wbc_scene = std::make_shared<WbcVelocityScene>(robot_model);
}

bool WbcVelocityTask::configureHook(){
    if (! WbcVelocityTaskBase::configureHook())
        return false;
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
