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
}


WbcVelocityTask::WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcVelocityTaskBase(name, engine){
}

WbcVelocityTask::~WbcVelocityTask(){
}

bool WbcVelocityTask::configureHook(){

    if (! WbcVelocityTaskBase::configureHook())
        return false;

    robot_model = new KinematicRobotModelKDL(_joint_names.get(), _base_frame.get());

    std::vector<RobotModelConfig> robot_model_config = _robot_models.get();
    for(size_t i = 0; i < robot_model_config.size(); i++){

        KDL::Tree tree;
        if(!kdl_parser::treeFromFile(robot_model_config[i].file, tree)){
            LOG_ERROR("Unable to parse urdf model from file %s", robot_model_config[i].file.c_str());
            return false;
        }

        if(!((KinematicRobotModelKDL*)robot_model)->addTree(tree, robot_model_config[i].hook, robot_model_config[i].initial_pose))
            return false;
    }

    LOG_DEBUG("... Configured Robot Model");

    solver = new HierarchicalLeastSquaresSolver();

    if(!solver->configure(WbcScene::getNConstraintVariablesPerPrio(_wbc_config.get()), robot_model->noOfJoints()))
        return false;

    ((HierarchicalLeastSquaresSolver*)solver)->setMaxSolverOutputNorm(_norm_max.get());
    ((HierarchicalLeastSquaresSolver*)solver)->setMinEigenvalue(_epsilon.get());
    if(_max_solver_output.get().size() > 0)
        ((HierarchicalLeastSquaresSolver*)solver)->setMaxSolverOutput(_max_solver_output.get());

    joint_weights = _initial_joint_weights.get();
    ((HierarchicalLeastSquaresSolver*)solver)->setJointWeights(joint_weights);

    LOG_DEBUG("... Configured Solver");

    wbc_scene = new WbcVelocityScene(robot_model, solver);
    if(!wbc_scene->configure(_wbc_config.get()))
        return false;

    LOG_DEBUG("... Configured WBC Scene");

    robot_vel.setZero(robot_model->jointNames().size());
    uint n_prios = wbc_scene->getConstraints().size();
    singular_values.resize(n_prios);
    inv_condition_numbers.resize(n_prios);
    damping.resize(n_prios);
    manipulability.resize(n_prios);

    return true;
}

bool WbcVelocityTask::startHook(){
    if (! WbcVelocityTaskBase::startHook())
        return false;
    return true;
}

void WbcVelocityTask::updateHook(){

    if(_joint_weights.readNewest(joint_weights) == RTT::NewData)
        ((HierarchicalLeastSquaresSolver*)solver)->setJointWeights(joint_weights);

    WbcVelocityTaskBase::updateHook();

    _current_joint_weights.write(joint_weights);

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

    delete wbc_scene;
    delete solver;
    delete robot_model;
}
