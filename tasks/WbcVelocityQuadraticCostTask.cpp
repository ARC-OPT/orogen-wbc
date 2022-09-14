/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcVelocityQuadraticCostTask.hpp"
#include <wbc/solvers/qpoases/QPOasesSolver.hpp>
#include <wbc/scenes/VelocitySceneQuadraticCost.hpp>
#include <wbc/core/RobotModelFactory.hpp>
#include <wbc/core/QPSolverFactory.hpp>
#include <wbc/core/PluginLoader.hpp>

using namespace wbc;

WbcVelocityQuadraticCostTask::WbcVelocityQuadraticCostTask(std::string const& name)
    : WbcVelocityQuadraticCostTaskBase(name){
}

WbcVelocityQuadraticCostTask::~WbcVelocityQuadraticCostTask(){
}

bool WbcVelocityQuadraticCostTask::configureHook(){
    PluginLoader::loadPlugin("libwbc-robot_models-" + _robot_model.get().type + ".so");
    robot_model =  std::shared_ptr<RobotModel>(RobotModelFactory::createInstance(_robot_model.get().type));

    PluginLoader::loadPlugin("libwbc-solvers-" + _qp_solver.get() + ".so");
    solver = std::shared_ptr<QPSolver>(QPSolverFactory::createInstance(_qp_solver.get()));

    wbc_scene = std::make_shared<VelocitySceneQuadraticCost>(robot_model, solver, this->getPeriod());
    std::dynamic_pointer_cast<VelocitySceneQuadraticCost>(wbc_scene)->setHessianRegularizer(_hessian_regularizer.get());
    compute_id = _compute_id.get();

    if (! WbcVelocityQuadraticCostTaskBase::configureHook())
        return false;

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
    PluginLoader::unloadPlugin("libwbc-robot_models-" + _robot_model.get().type + ".so");
}
