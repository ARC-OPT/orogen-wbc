/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcTask.hpp"
#include <wbc/Wbc.hpp>
#include <wbc/robot_models/RobotModel.hpp>
#include <wbc/solvers/Solver.hpp>

using namespace wbc;

WbcTask::WbcTask(std::string const& name)
    : WbcTaskBase(name)
{
    robot_model_interface = new RobotModelInterface(this);
}

WbcTask::WbcTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcTaskBase(name, engine)
{
    robot_model_interface = new RobotModelInterface(this);
}

WbcTask::~WbcTask()
{
    // clear interfaces
    delete robot_model_interface;

    ConstraintInterfaceMap::const_iterator it;
    for(it = constraint_interfaces.begin(); it != constraint_interfaces.end(); it++)
        delete it->second;
    constraint_interfaces.clear();
}

bool WbcTask::configureHook()
{
    if (! WbcTaskBase::configureHook())
        return false;

    // Get wbc config and check validity
    std::vector<ConstraintConfig> wbc_config = _wbc_config.get();
    if(!Wbc::isValid(wbc_config))
        return false;

    // Load robot Models and add task frames
    if(!robot_model->configure(_robot_models.get(),
                               wbc->getTaskFrameIDs(wbc_config),
                               _base_frame.get()))
        return false;

    robot_model_interface->configure(_robot_models.get());

    LOG_DEBUG("... Configured Robot Model");


    // Configure WBC
    if(!wbc->configure(wbc_config, robot_model->getJointNames()))
        return false;

    // Create constraint interfaces. Don't recreate existing interfaces.
    for(uint i = 0; i < wbc_config.size(); i++){
        if(constraint_interfaces.count(wbc_config[i].name) == 0)
            constraint_interfaces[wbc_config[i].name] = new ConstraintInterface(wbc_config[i].name, wbc, robot_model, this);
    }
    // Remove constraint interfaces that are not required anymore
    ConstraintInterfaceMap::const_iterator it;
    for(it = constraint_interfaces.begin(); it != constraint_interfaces.end(); it++){
        if(!wbc->hasConstraint(it->first))
            constraint_interfaces.erase(it->first);
    }

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
    ConstraintInterfaceMap::const_iterator it;
    for(it = constraint_interfaces.begin(); it != constraint_interfaces.end(); it++)
        it->second->reset();
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
    robot_model->update(joint_state, robot_model_interface->update());

    // Update constraints
    ConstraintInterfaceMap::const_iterator it;
    for(it = constraint_interfaces.begin(); it != constraint_interfaces.end(); it++)
        it->second->update();

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
}
