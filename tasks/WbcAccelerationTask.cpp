/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcAccelerationTask.hpp"
#include <wbc/scenes/AccelerationSceneTSID.hpp>
#include <wbc/solvers/qpoases/QPOasesSolver.hpp>
#include <wbc/core/RobotModelFactory.hpp>
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
    solver = std::make_shared<QPOASESSolver>();
    wbc_scene = std::make_shared<AccelerationSceneTSID>(robot_model, solver);

    if (! WbcAccelerationTaskBase::configureHook())
        return false;

    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(_n_wsr.get());
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(_solver_options.get());
    std::dynamic_pointer_cast<AccelerationSceneTSID>(wbc_scene)->setHessianRegularizer(_hessian_regularizer.get());

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
        std::shared_ptr<QPOASESSolver> qpoases_solver = std::dynamic_pointer_cast<QPOASESSolver>(solver);
        std::shared_ptr<AccelerationSceneTSID> wbc_acc_scene  = std::dynamic_pointer_cast<AccelerationSceneTSID>(wbc_scene);

        _solver_return_value.write(qpoases_solver->getReturnValue());
        double obj_func_value = qpoases_solver->getSQProblem().getObjVal();
        _obj_func_value.write(obj_func_value);
        _estimated_contact_wrenches.write(wbc_acc_scene->getContactWrenches());
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
