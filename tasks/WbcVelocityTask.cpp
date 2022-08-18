/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcVelocityTask.hpp"
#include <wbc/scenes/VelocityScene.hpp>
#include <wbc/solvers/hls/HierarchicalLSSolver.hpp>
#include "ConstraintInterface.hpp"
#include <wbc/core/RobotModelFactory.hpp>
#include <wbc/core/PluginLoader.hpp>

using namespace wbc;
using namespace std;

WbcVelocityTask::WbcVelocityTask(std::string const& name)
    : WbcVelocityTaskBase(name){
}

WbcVelocityTask::WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcVelocityTaskBase(name, engine){
}

bool WbcVelocityTask::configureHook(){
    if(robot_model.get() == 0){
        PluginLoader::loadPlugin("libwbc-robot_models-kdl.so");
        robot_model =  std::shared_ptr<RobotModel>(RobotModelFactory::createInstance("kdl"));
    }
    if(solver.get() == 0)
        solver = std::make_shared<HierarchicalLSSolver>();
    if(wbc_scene.get() == 0)
        wbc_scene = std::make_shared<VelocityScene>(robot_model, solver);

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
    PluginLoader::unloadPlugin("libwbc-robot_models-" + _robot_model.get().type + ".so");
}
