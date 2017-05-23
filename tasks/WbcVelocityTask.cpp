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

    robot_model = new KinematicRobotModelKDL();
    solver = new HierarchicalLeastSquaresSolver();
    wbc_scene = new WbcVelocityScene(robot_model, solver);
}



WbcVelocityTask::WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcVelocityTaskBase(name, engine){

    robot_model = new KinematicRobotModelKDL();
    solver = new HierarchicalLeastSquaresSolver();
    wbc_scene = new WbcVelocityScene(robot_model, solver);
}

WbcVelocityTask::~WbcVelocityTask(){

    delete robot_model;
    delete solver;
    delete wbc_scene;
}

bool WbcVelocityTask::configureHook(){

    if (! WbcVelocityTaskBase::configureHook())
        return false;

    ((HierarchicalLeastSquaresSolver*)solver)->setMaxSolverOutputNorm(_norm_max.get());
    ((HierarchicalLeastSquaresSolver*)solver)->setMinEigenvalue(_epsilon.get());
    if(_max_solver_output.get().size() > 0)
        ((HierarchicalLeastSquaresSolver*)solver)->setMaxSolverOutput(_max_solver_output.get());

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
        ((WbcVelocityScene*)wbc_scene)->evaluateConstraints( ((WbcVelocityScene*)wbc_scene)->getSolverOutput(), robot_vel);
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
