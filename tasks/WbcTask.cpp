/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcTask.hpp"
#include <wbc/Wbc.hpp>
#include <wbc/robot_models/RobotModel.hpp>
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

    // Load Models from file
    robot_model_interface = new RobotModelInterface(robot_model, this);
    std::vector<RobotModelConfig> robot_models = _robot_models.get();
    for(uint i = 0; i < robot_models.size(); i++){
        if(!robot_model->loadModel(robot_models[i]))
            return false;

        robot_model_interface->addPort(robot_models[i].hook);
    }

    // Check if wbc config is valid
    std::vector<ConstraintConfig> wbc_config = _wbc_config.get();
    for(size_t i = 0; i < wbc_config.size(); i++){
        if(!wbc_config[i].isValid())
            return false;
    }

    // Add Task Frames
    if(!robot_model->addTaskFrames(wbc->getTaskFrameIDs(wbc_config)))
        return false;


    LOG_DEBUG("... Configured Robot Model");


    // Configure WBC
    if(!wbc->configure(wbc_config, robot_model->getJointNames()))
        return false;

    // Create constraint interfaces
    for(uint i = 0; i < wbc_config.size(); i++)
        constraint_interfaces.push_back(new ConstraintInterface(wbc_config[i].name, wbc, robot_model,this));


    LOG_DEBUG("... Configured WBC");


    // Configure Solver
    if(!solver->configure(wbc->getConstraintVariablesPerPrio(), robot_model->getJointNames().size()))
        return false;


    LOG_DEBUG("... Configured Solver");


    ctrl_out.resize(robot_model->getJointNames().size());
    ctrl_out.names = robot_model->getJointNames();

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

    solver_output.setConstant(robot_model->getJointNames().size(), 0);

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

    // Update Robot Model
    robot_model_interface->update(joint_state);

    // Update constraints
    for(uint i = 0; i < constraint_interfaces.size(); i++)
        constraint_interfaces[i]->update();

    // Prepare opt. Problem
    const std::vector<TaskFrame*> &task_frames = robot_model->getTaskFrames();
    wbc->setupOptProblem(task_frames, opt_problem);

    // Solve opt. problem
    solver->solve(opt_problem, solver_output);

    //Write Task Frames
    task_frames_out.resize(task_frames.size());
    for(size_t i = 0; i < task_frames.size(); i++){
        task_frames_out[i] = *task_frames[i];
    }
    _task_frames.write(task_frames_out);

    // Write computation time for one cycle
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
std::vector<std::string> WbcTask::getJointNames(){
    if(robot_model)
        return robot_model->getJointNames();
    else
        return std::vector<std::string>();
}
