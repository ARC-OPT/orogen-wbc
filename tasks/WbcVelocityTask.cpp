/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcVelocityTask.hpp"
#include <base-logging/Logging.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <wbc/KinematicRobotModelKDL.hpp>
#include <wbc/WbcVelocityScene.hpp>
#include <wbc/HierarchicalLeastSquaresSolver.hpp>
#include "wbcTypes.hpp"

using namespace wbc;
using namespace std;

WbcVelocityTask::WbcVelocityTask(std::string const& name)
    : WbcVelocityTaskBase(name){
    robot_model = std::make_shared<KinematicRobotModelKDL>();
    solver = std::make_shared<HierarchicalLeastSquaresSolver>();
    wbc_scene = std::make_shared<WbcVelocityScene>(robot_model, solver);

}

WbcVelocityTask::WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcVelocityTaskBase(name, engine){
    robot_model = std::make_shared<KinematicRobotModelKDL>();
    solver = std::make_shared<HierarchicalLeastSquaresSolver>();
    wbc_scene = std::make_shared<WbcVelocityScene>(robot_model, solver);
}

WbcVelocityTask::~WbcVelocityTask(){    
    wbc_scene.reset();
    robot_model.reset();
    solver.reset();
}

bool WbcVelocityTask::configureHook(){
    if (! WbcVelocityTaskBase::configureHook())
        return false;

    std::shared_ptr<HierarchicalLeastSquaresSolver> hier_solver = std::static_pointer_cast<HierarchicalLeastSquaresSolver>(solver);

    hier_solver->setMaxSolverOutputNorm(_norm_max.get());
    hier_solver->setMinEigenvalue(_epsilon.get());
    if(_max_solver_output.get().size() > 0)
        hier_solver->setMaxSolverOutput(_max_solver_output.get());

    robot_vel.setZero(robot_model->jointNames().size());

    return true;
}

bool WbcVelocityTask::startHook(){
    if (! WbcVelocityTaskBase::startHook())
        return false;
    return true;
}

void WbcVelocityTask::updateHook(){

    WbcVelocityTaskBase::updateHook();

    //Compute debug data
    if(state() == RUNNING){
        for(uint i = 0; i <ctrl_out.size(); i++)
            robot_vel(i) = joint_state.getElementByName(ctrl_out.names[i]).speed;
        std::shared_ptr<WbcVelocityScene> wbc_vel_scene = std::static_pointer_cast<WbcVelocityScene>(wbc_scene);
        wbc_vel_scene->evaluateConstraints( wbc_vel_scene->getSolverOutput(), robot_vel);
    }
}

void WbcVelocityTask::stopHook(){

    //Set speed to zero
    for(uint i = 0; i < ctrl_out.size(); i++)
        ctrl_out[i].speed = 0.0;
    _ctrl_out.write(ctrl_out);

    WbcVelocityTaskBase::stopHook();
}

void WbcVelocityTask::cleanupHook()
{
    WbcVelocityTaskBase::cleanupHook();
}
