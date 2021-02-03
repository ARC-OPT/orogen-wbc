/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcAccelerationTask.hpp"
#include <wbc/robot_models/RobotModelHyrodyn.hpp>
#include <wbc/scenes/AccelerationSceneTSID.hpp>
#include <wbc/solvers/qpoases/QPOasesSolver.hpp>

using namespace wbc;

WbcAccelerationTask::WbcAccelerationTask(std::string const& name)
    : WbcAccelerationTaskBase(name){
    robot_model = std::make_shared<RobotModelHyrodyn>();
    solver = std::make_shared<QPOASESSolver>();
    wbc_scene = std::make_shared<AccelerationSceneTSID>(robot_model, solver);
}

WbcAccelerationTask::WbcAccelerationTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcAccelerationTaskBase(name, engine){
    robot_model = std::make_shared<RobotModelHyrodyn>();
    solver = std::make_shared<QPOASESSolver>();
    wbc_scene = std::make_shared<AccelerationSceneTSID>(robot_model, solver);
}

WbcAccelerationTask::~WbcAccelerationTask(){
}

bool WbcAccelerationTask::configureHook(){
    if (! WbcAccelerationTaskBase::configureHook())
        return false;

    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(_n_wsr.get());
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(_solver_options.get());

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
        _contact_wrenches.write(wbc_acc_scene->getContactWrenches());
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
}
