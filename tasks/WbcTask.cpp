/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcTask.hpp"
#include <wbc/core/Scene.hpp>
#include <wbc/core/RobotModel.hpp>
#include <wbc/core/QPSolver.hpp>
#include "ConstraintInterface.hpp"
#include <base-logging/Logging.hpp>

using namespace wbc;

WbcTask::WbcTask(std::string const& name)
    : WbcTaskBase(name),
      compute_id(false){
}

WbcTask::WbcTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcTaskBase(name, engine),
      compute_id(false){
}

WbcTask::~WbcTask(){
}

bool WbcTask::configureHook(){
    if (! WbcTaskBase::configureHook())
        return false;
    if(!robot_model->configure(_robot_model.get()))
            return false;

    LOG_DEBUG("... Configured Robot Model");

    wbc_config = _wbc_config.get();
    if(!wbc_scene->configure(wbc_config))
        return false;

    LOG_DEBUG("... Configured WBC Scene");

    // Create constraint interfaces. Don't recreate existing interfaces.
    for(ConstraintConfig cfg : wbc_config){
        if(constraint_interfaces.count(cfg.name) == 0)
            constraint_interfaces[cfg.name] = std::make_shared<ConstraintInterface>(cfg, wbc_scene, robot_model, this);
        else
            constraint_interfaces[cfg.name]->cfg = cfg;
    }

    // Remove constraint interfaces that are not required anymore
    for(const auto &it : constraint_interfaces){
        if(!wbc_scene->hasConstraint(it.first))
            constraint_interfaces.erase(it.first);
    }

    floating_base_state = robot_model->floatingBaseState();
    joint_weights = _initial_joint_weights.get();

    LOG_DEBUG("... Created ports");

    compute_constraint_status = _compute_constraint_status.get();
    integrate = _integrate.get();

    use_ankle_admittance = _use_ankle_admittance.get();
    _ankle_admittance_cop_threshold_x = _aa_cop_x_threshold.get();
    _ankle_admittance_cop_threshold_y = _aa_cop_y_threshold.get();
    _ankle_admittance_gain_roll = _aa_gain_roll.get();
    _ankle_admittance_gain_pitch = _aa_gain_pitch.get();
    _ankle_admittance_max_speed = _aa_max_speed.get();

    return true;
}

bool WbcTask::startHook(){
    if (! WbcTaskBase::startHook())
        return false;

    //Clear all task references, weights etc. to have to secure initial state
    for(const auto &cfg : wbc_config)
        wbc_scene->getConstraint(cfg.name)->reset();
    stamp.microseconds = 0;
    timing_stats.desired_period = this->getPeriod();

    return true;
}


Eigen::Matrix4d f1_transform_f2(const wbc::RobotModelPtr robot_model, const std::string& f1_name, const std::string& f2_name){

    static Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
    static Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
    static Eigen::Matrix4d T3;

    auto f1 = robot_model->rigidBodyState("world", f1_name);
    auto f2 = robot_model->rigidBodyState("world", f2_name);

    T1.topLeftCorner<3,3>() = f1.pose.orientation.toRotationMatrix();
    T1.col(3).head<3>() = f1.pose.position;

    T2.topLeftCorner<3,3>() = f2.pose.orientation.toRotationMatrix();
    T2.col(3).head<3>() = f2.pose.position;

    T3 = T1.inverse() * T2;

    return T3;
}

base::Matrix3d cross_mtx(const base::Vector3d& vec)
{
    base::Matrix3d mtx;
    mtx.setZero();
    mtx <<  0.0, -vec(2), vec(1),
            vec(2), 0.0, -vec(0),
            -vec(1), vec(0), 0.0;
    return mtx;
}

base::MatrixXd toDualActionMatrix(const base::Matrix4d& transform){ // considering force, torque ordering
    
    base::MatrixXd dual_action_mtx = base::MatrixXd::Zero(6,6);
    dual_action_mtx.topLeftCorner<3,3>() = transform.topLeftCorner<3,3>();
    dual_action_mtx.bottomRightCorner<3,3>() = transform.topLeftCorner<3,3>();
    dual_action_mtx.bottomLeftCorner<3,3>() = cross_mtx(transform.col(3).head<3>()) * transform.topLeftCorner<3,3>();
    return dual_action_mtx;
}

void WbcTask::apply_ankle_stabilization(const wbc::RobotModelPtr robot_model, base::samples::Joints& output_joints, const base::Wrench& wrench, 
                const std::string& force_frame, const std::string& support_frame, 
                const std::string& joint_name_roll, const std::string& joint_name_pitch, double dt)
{
    // step 1 -> wrenches are local oriented in ft frame
    base::VectorXd sc_wrench(6), e_wrench(6);
    e_wrench << wrench.force, wrench.torque;
    
    // step 2 -> transform forces in support center frames
    base::Matrix4d ft_to_fsc = f1_transform_f2(robot_model, support_frame, force_frame); // compute {f1}^T_{f2}
    sc_wrench = toDualActionMatrix(ft_to_fsc) * e_wrench;

    // std::cerr << "period " << dt << std::endl;
    // std::cerr << "transform\n" << ft_to_fsc << std::endl;
    // std::cerr << "dual matrix\n" << toDualActionMatrix(ft_to_fsc) << std::endl;

    // step 2 special ->  get minimu torque threshold based on cop offset threashold
    base::Vector3d cop_th(_ankle_admittance_cop_threshold_x, _ankle_admittance_cop_threshold_y, 0.0);
    base::Vector2d torque_th = cop_th.cross(sc_wrench.head<3>()).head<2>().cwiseAbs();

    // get the torque we can remove as it is under threshold
    base::Vector2d rp_torques_intra = sc_wrench.segment<2>(3);
    rp_torques_intra = rp_torques_intra.cwiseMin(torque_th);
    rp_torques_intra = rp_torques_intra.cwiseMax(-torque_th);

    // remove under threshold torque from overall torque
    base::Vector2d rp_torques = sc_wrench.segment<2>(3) - rp_torques_intra;

    // step 3 -> compute roll and pitch offset and rotate it in world frame (to be used as task reference)
    double roll = _ankle_admittance_gain_roll * rp_torques(0);
    double pitch = _ankle_admittance_gain_pitch * rp_torques(1);

    // step 3 special -> cut output
    double max_displacement = _ankle_admittance_max_speed * dt; // not over 0.2 rad/s
    roll = std::max(std::min(max_displacement, roll), -max_displacement); //
    pitch = std::max(std::min(max_displacement, pitch), -max_displacement);

    output_joints[joint_name_roll].position += roll;
    output_joints[joint_name_roll].speed += roll / dt;

    output_joints[joint_name_pitch].position += pitch;
    output_joints[joint_name_pitch].speed += pitch / dt;
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
    if(state() != RUNNING)
        state(RUNNING);

    _floating_base_state.readNewest(floating_base_state);
    if(_floating_base_state_deprecated.readNewest(floating_base_state_rbs) == RTT::NewData){
        floating_base_state.pose.position = floating_base_state_rbs.position;
        floating_base_state.pose.orientation = floating_base_state_rbs.orientation;
        floating_base_state.twist.linear = floating_base_state_rbs.velocity;
        floating_base_state.twist.angular = floating_base_state_rbs.angular_velocity;
        floating_base_state.time = floating_base_state_rbs.time;
        floating_base_state.frame_id = floating_base_state_rbs.targetFrame;
    }

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
    for(const auto& it : constraint_interfaces)
        it.second->update();
    wbc_scene->setJointWeights(joint_weights);
    timing_stats.time_constraint_update = (base::Time::now()-cur_time).toSeconds();

    // Update Quadratic program
    cur_time = base::Time::now();
    hierarchical_qp = wbc_scene->update();
    timing_stats.time_scene_update = (base::Time::now()-cur_time).toSeconds();

    // Solve
    cur_time = base::Time::now();
    _current_joint_weights.write(wbc_scene->getActuatedJointWeights());
    solver_output_joints = wbc_scene->solve(hierarchical_qp);
    if(integrate)
        integrator.integrate(robot_model->jointState(robot_model->jointNames()), solver_output_joints, this->getPeriod());
    if(compute_id)
        robot_model->computeInverseDynamics(solver_output_joints);

    if(use_ankle_admittance) {
        double dt = this->getPeriod();
        base::Wrench left_ankle_wrench = contact_wrenches[ "LLAnkle_ft"];
        base::Wrench right_ankle_wrench = contact_wrenches[ "LRAnkle_ft"];
        apply_ankle_stabilization(robot_model, solver_output_joints, left_ankle_wrench, "LLAnkle_FT", "FL_SupportCenter", "LLAnkleRoll", "LLAnklePitch", dt);
        apply_ankle_stabilization(robot_model, solver_output_joints, right_ankle_wrench, "LRAnkle_FT", "FR_SupportCenter", "LRAnkleRoll", "LRAnklePitch", dt);
    }

    _solver_output.write(solver_output_joints);
    timing_stats.time_solve = (base::Time::now()-cur_time).toSeconds();

    // Write debug output
    _full_joint_state.write(robot_model->jointState(robot_model->jointNames()));
    _current_qp.write(hierarchical_qp);
    full_joint_state = robot_model->jointState(robot_model->jointNames());
    if(compute_constraint_status){
        constraints_status = wbc_scene->updateConstraintsStatus();
        for(const auto &c : constraint_interfaces)
            c.second->writeConstraintStatus(constraints_status[c.first]);
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
}

void WbcTask::activateConstraint(const std::string& constraint_name, double activation){
    wbc_scene->getConstraint(constraint_name)->setActivation(activation);
}

void WbcTask::activateConstraints(const std::vector<std::string>& constraint_names, double activation){
    for(auto name : constraint_names)
         wbc_scene->getConstraint(name)->setActivation(activation);
}

void WbcTask::deactivateAllConstraints(){
    for(auto constraint : wbc_config)
        wbc_scene->getConstraint(constraint.name)->setActivation(0);
}

