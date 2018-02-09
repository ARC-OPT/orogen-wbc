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

void  WbcVelocityTask::updateConstraints(){
    if(state() == RUNNING){
        std::shared_ptr<WbcVelocityScene> wbc_vel_scene = std::static_pointer_cast<WbcVelocityScene>(wbc_scene);

        constraints_prio.time = robot_model->lastUpdate();
        constraints_prio.joint_names = robot_model->jointNames();
        constraints_prio.constraints = wbc_vel_scene->getConstraintsByPrio();
        _constraints_prio.write(constraints_prio);
    }
}
