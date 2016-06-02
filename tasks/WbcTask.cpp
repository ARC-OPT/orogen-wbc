/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcTask.hpp"
#include <wbc/Wbc.hpp>
#include <wbc/models/RobotModel.hpp>
#include <wbc/solvers/Solver.hpp>

using namespace wbc;

WbcTask::WbcTask(std::string const& name)
    : WbcTaskBase(name)
{
}

WbcTask::WbcTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcTaskBase(name, engine)
{
}

WbcTask::~WbcTask()
{
}

bool WbcTask::configureHook()
{
    if (! WbcTaskBase::configureHook())
        return false;

    // Configure WBC
    std::vector<ConstraintConfig> wbc_config = _wbc_config.get();
    std::vector<std::string> joint_names = _joint_names.get();
    if(!wbc->configure(wbc_config, joint_names))
        return false;

    // Create constraint interfaces
    for(uint i = 0; i < wbc_config.size(); i++)
        constraint_interfaces.push_back(new ConstraintInterface(wbc->getConstraint(wbc_config[i].name), this));

    LOG_DEBUG("... Configured WBC");


    // Load Models
    std::string model_file = _robot_model_file.get();
    std::vector<RobotModelFromFile> robot_models = _robot_models.get();
    if(!model_file.empty()){
        base::samples::RigidBodyState dummy;
        if(!robot_model->addModelFromFile(model_file, dummy))
            return false;
    }

    robot_model_interface = new RobotModelInterface(robot_model, this);
    if(!robot_models.empty()){
        for(uint i = 0; i < robot_models.size(); i++){
            if(!robot_model->addModelFromFile(robot_models[i].file, robot_models[i].initial_pose, robot_models[i].hook))
                return false;

            robot_model_interface->addPort(robot_models[i].hook);
        }
    }

    // Add task frames
    if(!robot_model->addTaskFrames(wbc->getTaskFrameIDs()))
        return false;

    LOG_DEBUG("... Configured Robot Models");


    // Configure Solver
    if(!solver->configure(wbc->getNumberOfConstraintsPerPriority(), wbc->getJointNames().size()))
        return false;

    LOG_DEBUG("... Configured Solver");

    ctrl_out.resize(wbc->getJointNames().size());
    ctrl_out.names = wbc->getJointNames();

    return true;
}

bool WbcTask::startHook()
{
    if (! WbcTaskBase::startHook())
        return false;

    //Clear all task references, weights etc. to have to secure initial state
    for(uint i = 0; i < constraint_interfaces.size(); i++)
        constraint_interfaces[i]->reset();
    stamp.microseconds = 0;

    return true;
}
void WbcTask::updateHook()
{
    WbcTaskBase::updateHook();

    // Compute cycle time
    base::Time cur = base::Time::now();
    if(!stamp.isNull())
        _actual_cycle_time.write((cur - stamp).toSeconds());
    stamp = cur;

    // Read joint status
    if(_joint_state.readNewest(joint_state) == RTT::NoData){
        if(state() != NO_JOINT_STATE)
            state(NO_JOINT_STATE);
        return;
    }

    if(state() != RUNNING)
        state(RUNNING);

    // Update constraints
    for(uint i = 0; i < constraint_interfaces.size(); i++)
        constraint_interfaces[i]->update(joint_state);

    //Update Robot Model
    robot_model_interface->update(joint_state);

    // Prepare opt. Problem
    wbc->setupOptProblem(robot_model->getTaskFrames(), opt_problem);

    // Solve opt. problem
    solver->solve(opt_problem, solver_output);

    _computation_time.write((base::Time::now() - cur).toSeconds());
}

void WbcTask::errorHook()
{
    WbcTaskBase::errorHook();
}
void WbcTask::stopHook()
{
    WbcTaskBase::stopHook();
}
void WbcTask::cleanupHook()
{
    WbcTaskBase::cleanupHook();

    // clear interfaces
    delete robot_model_interface;
    for(uint i = 0; i < constraint_interfaces.size(); i++)
        delete constraint_interfaces[i];
    constraint_interfaces.clear();
}
