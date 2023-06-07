/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcTask.hpp"
#include <wbc/core/Scene.hpp>
#include <wbc/core/RobotModel.hpp>
#include <wbc/core/QPSolver.hpp>
#include "TaskInterface.hpp"
#include <base-logging/Logging.hpp>
#include <wbc/core/PluginLoader.hpp>

using namespace wbc;

WbcTask::WbcTask(std::string const& name)
    : WbcTaskBase(name){
}

WbcTask::WbcTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcTaskBase(name, engine){
}

WbcTask::~WbcTask(){
}

bool WbcTask::configureHook(){
    if (! WbcTaskBase::configureHook())
        return false;

    PluginLoader::loadPlugin("libwbc-robot_models-" + _robot_model.get().type + ".so");
    robot_model =  std::shared_ptr<RobotModel>(RobotModelFactory::createInstance(_robot_model.get().type));

    PluginLoader::loadPlugin("libwbc-solvers-" + _qp_solver.get() + ".so");
    solver = std::shared_ptr<QPSolver>(QPSolverFactory::createInstance(_qp_solver.get()));

    PluginLoader::loadPlugin("libwbc-scenes-" + _wbc_type.get() + ".so");
    wbc_scene = std::shared_ptr<Scene>(SceneFactory::createInstance(_wbc_type.get(), robot_model, solver, this->getPeriod()));

    if(!robot_model->configure(_robot_model.get()))
            return false;

    LOG_DEBUG("... Configured Robot Model");

    wbc_config = _wbc_config.get();
    if(!wbc_scene->configure(wbc_config))
        return false;

    LOG_DEBUG("... Configured WBC Scene");

    // Create task interfaces. Don't recreate existing interfaces.
    for(TaskConfig cfg : wbc_config){
        if(task_interfaces.count(cfg.name) == 0)
            task_interfaces[cfg.name] = std::make_shared<TaskInterface>(cfg, wbc_scene, robot_model, this);
        else
            task_interfaces[cfg.name]->cfg = cfg;
    }

    // Remove task interfaces that are not required anymore
    for(const auto &it : task_interfaces){
        if(!wbc_scene->hasTask(it.first))
            task_interfaces.erase(it.first);
    }

    floating_base_state = robot_model->floatingBaseState();
    joint_weights = _initial_joint_weights.get();

    LOG_DEBUG("... Created ports");

    compute_task_status = _compute_task_status.get();
    integrate = _integrate.get();
    has_floating_base_state = false;

    return true;
}

bool WbcTask::startHook(){
    if (! WbcTaskBase::startHook())
        return false;

    //Clear all task references, weights etc. to have to secure initial state
    for(const auto &cfg : wbc_config)
        wbc_scene->getTask(cfg.name)->reset();
    stamp.microseconds = 0;
    timing_stats.desired_period = this->getPeriod();

    return true;
}

void WbcTask::updateHook(){
    // Compute cycle time
    base::Time start_time = base::Time::now();
    if(!stamp.isNull())
        timing_stats.actual_period = (start_time - stamp).toSeconds();
    stamp = start_time;

    WbcTaskBase::updateHook();

    _joint_state.readNewest(joint_state);
    if(joint_state.empty()){
        if(state() != NO_JOINT_STATE)
            state(NO_JOINT_STATE);
        return;
    }

    if(robot_model->hasFloatingBase()){
        if(_floating_base_state.readNewest(floating_base_state) == RTT::NewData)
            has_floating_base_state = true;
        if(_floating_base_state_deprecated.readNewest(floating_base_state_rbs) == RTT::NewData){
            floating_base_state.pose.position = floating_base_state_rbs.position;
            floating_base_state.pose.orientation = floating_base_state_rbs.orientation;
            floating_base_state.twist.linear = floating_base_state_rbs.velocity;
            floating_base_state.twist.angular = floating_base_state_rbs.angular_velocity;
            floating_base_state.acceleration.setZero();
            floating_base_state.time = floating_base_state_rbs.time;
            floating_base_state.frame_id = floating_base_state_rbs.targetFrame;
            has_floating_base_state = true;
        }

        if(!has_floating_base_state){
            if(state() != NO_FLOATING_BASE_STATE)
                state(NO_FLOATING_BASE_STATE);
            return;
        }

    }

    if(state() != RUNNING)
        state(RUNNING);

    if(_active_contacts.readNewest(active_contacts) == RTT::NewData)
        robot_model->setActiveContacts(active_contacts);

    if(_contact_wrenches.readNewest(contact_wrenches) == RTT::NewData)
        robot_model->setContactWrenches(contact_wrenches);

    // Update Robot Model
    base::Time cur_time = base::Time::now();
    robot_model->update(joint_state, floating_base_state);
    _com.write(robot_model->centerOfMass());
    timing_stats.time_robot_model_update = (base::Time::now()-cur_time).toSeconds();

    // Update Scene
    _joint_weights.readNewest(joint_weights);
    cur_time = base::Time::now();
    for(const auto& it : task_interfaces)
        it.second->update();
    wbc_scene->setJointWeights(joint_weights);
    timing_stats.time_task_update = (base::Time::now()-cur_time).toSeconds();

    // Update Quadratic program
    cur_time = base::Time::now();
    hierarchical_qp = wbc_scene->update();
    timing_stats.time_scene_update = (base::Time::now()-cur_time).toSeconds();

    // Solve
    cur_time = base::Time::now();
    _current_joint_weights.write(wbc_scene->getActuatedJointWeights());
    solver_output_joints = wbc_scene->solve(hierarchical_qp);
    if(integrate)
        integrator.integrate(robot_model->jointState(robot_model->actuatedJointNames()), solver_output_joints, this->getPeriod());
    _solver_output.write(solver_output_joints);
    timing_stats.time_solve = (base::Time::now()-cur_time).toSeconds();

    // Write debug output
    _full_joint_state.write(robot_model->jointState(robot_model->jointNames()));
    _current_qp.write(hierarchical_qp);
    full_joint_state = robot_model->jointState(robot_model->jointNames());
    if(compute_task_status){
        tasks_status = wbc_scene->updateTasksStatus();
        for(const auto &c : task_interfaces)
            c.second->writeTaskStatus(tasks_status[c.first]);
    }

    timing_stats.time_per_cycle = (base::Time::now()-start_time).toSeconds();
    _timing_stats.write(timing_stats);
}

void WbcTask::errorHook(){
    WbcTaskBase::errorHook();
}

void WbcTask::stopHook(){
    WbcTaskBase::stopHook();
}

void WbcTask::cleanupHook(){
    WbcTaskBase::cleanupHook();
    joint_state.clear();
    solver_output_joints.clear();
    full_joint_state.clear();
    integrator.reinit();

    PluginLoader::unloadPlugin("libwbc-robot_models-" + _robot_model.get().type + ".so");
    PluginLoader::unloadPlugin("libwbc-solvers-" + _qp_solver.get() + ".so");
    PluginLoader::unloadPlugin("libwbc-scenes-" + _wbc_type.get() + ".so");

    RobotModelFactory::clear();
    QPSolverFactory::clear();
    SceneFactory::clear();
}

void WbcTask::activateTask(const std::string& task_name, double activation){
    wbc_scene->getTask(task_name)->setActivation(activation);
}

void WbcTask::activateTasks(const std::vector<std::string>& task_names, double activation){
    for(auto name : task_names)
         wbc_scene->getTask(name)->setActivation(activation);
}

void WbcTask::deactivateAllTasks(){
    for(auto task : wbc_config)
        wbc_scene->getTask(task.name)->setActivation(0);
}

