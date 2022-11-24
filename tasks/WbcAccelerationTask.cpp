/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcAccelerationTask.hpp"
#include <wbc/scenes/AccelerationSceneReducedTSID.hpp>
#include <wbc/scenes/AccelerationSceneTSID.hpp>
#include <wbc/core/RobotModelFactory.hpp>
#include <wbc/core/QPSolverFactory.hpp>
#include <wbc/core/PluginLoader.hpp>

using namespace wbc;

WbcAccelerationTask::WbcAccelerationTask(std::string const& name)
    : WbcAccelerationTaskBase(name){
}

WbcAccelerationTask::WbcAccelerationTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcAccelerationTaskBase(name, engine){
}

WbcAccelerationTask::~WbcAccelerationTask(){
}

bool WbcAccelerationTask::configureHook(){
    PluginLoader::loadPlugin("libwbc-robot_models-" + _robot_model.get().type + ".so");
    robot_model =  std::shared_ptr<RobotModel>(RobotModelFactory::createInstance(_robot_model.get().type));

    PluginLoader::loadPlugin("libwbc-solvers-" + _qp_solver.get() + ".so");
    solver = std::shared_ptr<QPSolver>(QPSolverFactory::createInstance(_qp_solver.get()));

    reduced = _reduced.get();

    if(reduced){
        wbc_scene = std::make_shared<AccelerationSceneReducedTSID>(robot_model, solver, this->getPeriod());
        std::dynamic_pointer_cast<AccelerationSceneReducedTSID>(wbc_scene)->setHessianRegularizer(_hessian_regularizer.get());
    }else{
        wbc_scene = std::make_shared<AccelerationSceneTSID>(robot_model, solver, this->getPeriod());
        std::dynamic_pointer_cast<AccelerationSceneTSID>(wbc_scene)->setHessianRegularizer(_hessian_regularizer.get());
    }

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

    if(solver_output_joints.size() > 0){
        if(reduced){
            std::shared_ptr<AccelerationSceneReducedTSID> wbc_acc_scene  = std::dynamic_pointer_cast<AccelerationSceneReducedTSID>(wbc_scene);
            _estimated_contact_wrenches.write(wbc_acc_scene->getContactWrenches());
        }else{
            std::shared_ptr<AccelerationSceneTSID> wbc_acc_scene  = std::dynamic_pointer_cast<AccelerationSceneTSID>(wbc_scene);
            _estimated_contact_wrenches.write(wbc_acc_scene->getContactWrenches());
        }
    }
}

void WbcAccelerationTask::errorHook(){
    WbcAccelerationTaskBase::errorHook();
}

void WbcAccelerationTask::stopHook(){
    WbcAccelerationTaskBase::stopHook();
}

void WbcAccelerationTask::cleanupHook(){
    WbcAccelerationTaskBase::cleanupHook();
    PluginLoader::unloadPlugin("libwbc-robot_models-" + _robot_model.get().type + ".so");
}
