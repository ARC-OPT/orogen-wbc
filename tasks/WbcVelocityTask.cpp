/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include <wbc/SubTask.hpp>
#include "WbcVelocityTask.hpp"
#include <wbc/TaskFrame.hpp>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <fstream>

using namespace wbc;
using namespace std;

WbcVelocityTask::WbcVelocityTask(std::string const& name)
    : WbcVelocityTaskBase(name){
}

WbcVelocityTask::WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcVelocityTaskBase(name, engine){
}

bool WbcVelocityTask::configureHook(){
    if (! WbcVelocityTaskBase::configureHook())
        return false;

    std::string urdf_file = _urdf.get();
    std::vector<wbc::SubTaskConfig> wbc_config = _wbc_config.get();
    task_timeout_ = _task_timeout.get();
    write_debug_ = _write_debug.get();

    //
    // Load urdf model: Kinematics of the robot
    //
    std::ifstream urdf_stream(urdf_file.c_str());
    if(!urdf_stream.is_open()){
        LOG_ERROR("Error opening urdf file %s", urdf_file.c_str());
        return false;
    }
    std::string xml((std::istreambuf_iterator<char>(urdf_stream)), std::istreambuf_iterator<char>());
    boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(xml);
    LOG_DEBUG("Parsing URDF file done");

    //
    // Create ports
    //
    for(uint i = 0; i < wbc_config.size(); i++)
    {
        SubTaskConfig conf = wbc_config[i];
        SubTaskInterface* sti = new SubTaskInterface(conf);
        if(_tasks_active.value())
            sti->activation = 1;
        else
            sti->activation = 0;

        addPortsForSubTask(sti);
        sub_task_interface_map_[conf.name] = sti;
    }

    //
    // Configure wbc lib
    //
    KDL::Tree full_tree, tree;
    if(!kdl_parser::treeFromFile(urdf_file, full_tree)){
        LOG_ERROR("Unable to load KDL Tree from urdf file: %s", urdf_file.c_str());
        return false;
    }
    std::vector<wbc::SubChainConfig> reduced_tree = _reduced_tree.get();
    if(reduced_tree.empty())
        tree = full_tree;
    else
    {
        for(uint i = 0; i < reduced_tree.size(); i++)
        {
            KDL::Chain chain;
            if(!full_tree.getChain(reduced_tree[i].root, reduced_tree[i].tip, chain))
            {
                LOG_ERROR("Could not extract sub chain between %s and %s from KDL tree", reduced_tree[i].root.c_str(), reduced_tree[i].tip.c_str());
                return false;
            }
            //Root of first subchain will be root of whole KDL tree
            if(i == 0)
                tree.addSegment(KDL::Segment(reduced_tree[i].root, KDL::Joint(reduced_tree[i].root,KDL::Joint::None),KDL::Frame::Identity()), "root");
            tree.addChain(chain, reduced_tree[i].root);
        }
    }
    if(!wbc_.configure(tree, wbc_config, _joint_names.get()))
        return false;

    LOG_DEBUG("Configuring WBC Config done");

    //
    // Configure Solver
    //
    solver_.setNormMax(_norm_max.get());
    if(!solver_.configure(wbc_.no_task_vars_pp_, wbc_.no_robot_joints_))
        return false;

    base::VectorXd joint_weight_vect = _initial_joint_weights.get();
    if(joint_weight_vect.size() != 0)
    {
        base::MatrixXd joint_weights(joint_weight_vect.size(), joint_weight_vect.size());
        joint_weights.setIdentity();
        joint_weights.diagonal() = joint_weight_vect;
        solver_.setJointWeights(joint_weights);
    }
    solver_.setSVDMethod(_svd_method.get());

    LOG_DEBUG("Configuring Solver Config done");

    solver_output_.resize(wbc_.no_robot_joints_);
    solver_output_.setZero();
    act_robot_velocity_.resize(wbc_.no_robot_joints_);
    act_robot_velocity_.setZero();
    joint_weights_ = base::VectorXd::Ones(wbc_.no_robot_joints_);
    joint_weight_mat_.resize(wbc_.no_robot_joints_, wbc_.no_robot_joints_);
    joint_weight_mat_.setIdentity();
    ctrl_out_.resize(wbc_.no_robot_joints_);
    ctrl_out_.names = wbc_.jointNames();

    return true;
}

bool WbcVelocityTask::startHook(){
    if (! WbcVelocityTaskBase::startHook())
        return false;
    return true;
}

void WbcVelocityTask::updateHook(){
    WbcVelocityTaskBase::updateHook();

    base::Time start = base::Time::now();
    base::Time sstart = base::Time::now();

    //
    // Read inputs
    //
    if(_joint_state.read(joint_status_) == RTT::NoData){
        LOG_DEBUG("No data on joint status port");
        return;
    }

    for(SubTaskInterfaceMap::iterator it = sub_task_interface_map_.begin(); it != sub_task_interface_map_.end(); it++)
        it->second->update();

    if(_joint_weights.read(joint_weights_) == RTT::NewData)
    {
        joint_weight_mat_.diagonal() = joint_weights_;
        solver_.setJointWeights((Eigen::MatrixXd& )joint_weight_mat_);
    }

    LOG_DEBUG("Read ports: %f", (base::Time::now() - start).toSeconds());
    start = base::Time::now();

    //
    // Set task weights and reference values
    //
    for(SubTaskInterfaceMap::iterator it = sub_task_interface_map_.begin(); it != sub_task_interface_map_.end(); it++)
    {
        SubTask* sub_task = wbc_.subTask(it->first);
        SubTaskInterface *iface = it->second;

        if(!base::isUnset(task_timeout_))
        {
            double diff = (base::Time::now() - iface->last_task_input).toSeconds();

            if( (diff > task_timeout_) &&
               !(iface->task_timed_out_))
                LOG_DEBUG("Task %s has timed out! No new reference has been received for %f seconds. Timeout is %f seconds", iface->config.name.c_str(), diff, task_timeout_);

            if(diff > task_timeout_)
                iface->task_timed_out_ = 1;
            else
                iface->task_timed_out_ = 0;
        }
        sub_task->task_weights_.diagonal() = iface->weights * iface->activation * (!iface->task_timed_out_);
        sub_task->y_des_ = iface->y_des * (!iface->task_timed_out_);
    }

    LOG_DEBUG("Set task weights: %f", (base::Time::now() - start).toSeconds());
    start = base::Time::now();

    //
    // Update equation system
    //
    wbc_.update(joint_status_);

    LOG_DEBUG("Update equation system: %f", (base::Time::now() - start).toSeconds());
    start = base::Time::now();

    //
    // Compute control solution
    //
    for(uint i = 0; i < wbc_.Wy_.size(); i++)
        solver_.setTaskWeights(wbc_.Wy_[i], i);

    solver_.solve(wbc_.A_, wbc_.y_ref_, (Eigen::VectorXd& )solver_output_);

    LOG_DEBUG("Solve: %f", (base::Time::now() - start).toSeconds());
    start = base::Time::now();

    //
    // Write output
    //
    if(ctrl_out_.empty()){
        ctrl_out_.resize(joint_status_.size());
        ctrl_out_.names = joint_status_.names;
    }
    for(uint i = 0; i < ctrl_out_.size(); i++){
        uint idx = wbc_.joint_index_map_[ctrl_out_.names[i]];
        ctrl_out_[i].speed = solver_output_(idx);
    }
    ctrl_out_.time = base::Time::now();
    _ctrl_out.write(ctrl_out_);


    LOG_DEBUG("Write Output: %f", (base::Time::now() - start).toSeconds());
    start = base::Time::now();

    //
    // write Debug Data
    //
    _damping.write(solver_.getCurDamping());
    _sample_time.write((base::Time::now() - sstart).toSeconds());
    for(SubTaskInterfaceMap::iterator it = sub_task_interface_map_.begin(); it != sub_task_interface_map_.end(); it++)
    {
        SubTask* task = wbc_.subTask(it->first);
        SubTaskInterface *iface = it->second;
        if(task->config_.type == wbc::task_type_cartesian)
        {
            base::samples::RigidBodyState rbs;
            kdl_conversions::KDL2RigidBodyState(task->pose_, rbs);
            rbs.time = base::Time::now();
            rbs.sourceFrame = task->tf_root_->tf_name_;
            rbs.targetFrame = task->tf_tip_->tf_name_;

            iface->pose_out_port->write(rbs);
        }
    }
    if(write_debug_)
    {
        for(SubTaskInterfaceMap::iterator it = sub_task_interface_map_.begin(); it != sub_task_interface_map_.end(); it++)
        {
            SubTask* task = wbc_.subTask(it->first);
            SubTaskInterface *iface = it->second;
            for(size_t i = 0; i < ctrl_out_.size(); i++)
            {
                size_t idx = joint_status_.mapNameToIndex(ctrl_out_.names[i]);
                act_robot_velocity_[i] = joint_status_[idx].speed;
            }

            iface->A_task_out_port->write(wbc_.subTask(it->first)->A_);
            iface->y_solution_out_port->write(wbc_.subTask(it->first)->A_ * solver_output_);
            iface->y_act_out_port->write(wbc_.subTask(it->first)->A_ * act_robot_velocity_);
        }
    }

    LOG_DEBUG("Write all Debug: %f", (base::Time::now() - start).toSeconds());
    LOG_DEBUG("Total: %f\n", (base::Time::now() - sstart).toSeconds());
}

void WbcVelocityTask::cleanupHook()
{
    WbcVelocityTaskBase::cleanupHook();

    for(SubTaskInterfaceMap::iterator it = sub_task_interface_map_.begin(); it != sub_task_interface_map_.end(); it++)
    {
        removePortsOfSubTask(it->second);
        delete it->second;
    }
    sub_task_interface_map_.clear();
}


SubTaskInterface::SubTaskInterface(const SubTaskConfig& conf)
{
    std::string port_namespace;
    uint no_task_vars;
    config = conf;
    task_timed_out_ = 0;

    if(config.type == wbc::task_type_cartesian){
        no_task_vars = 6;
        std::stringstream ss;
        ss<<"p"<<config.priority<<"_cart_"<<config.name;
        port_namespace = ss.str();

        pose_out_port = new RTT::OutputPort<base::samples::RigidBodyState>("pose_" + port_namespace);
        cart_ref_port = new RTT::InputPort<base::samples::RigidBodyState>("ref_" + port_namespace);
        jnt_ref_port = 0;
        kdl_conversions::KDL2RigidBodyState(KDL::Twist::Zero(), cart_ref);
    }
    else{
        no_task_vars = config.joint_names.size();
        std::stringstream ss;
        ss<<"p"<<config.priority<<"_jnt_"<<config.name;
        port_namespace = ss.str();

        jnt_ref_port = new RTT::InputPort<base::samples::Joints>("ref_" + port_namespace);
        cart_ref_port = 0;
        pose_out_port = 0;
        jnt_ref.resize(config.joint_names.size());
        jnt_ref.names = config.joint_names;
        for(uint i = 0; i < config.joint_names.size(); i++)
            jnt_ref[i].speed = 0;
    }

    y_des.resize(no_task_vars);
    y_des.setZero();

    last_task_input = base::Time::now();
    weights = base::VectorXd::Ones(no_task_vars);

    activation_port = new RTT::InputPort<double>("activation_" + port_namespace);
    weight_port = new RTT::InputPort<base::VectorXd>("weight_" + port_namespace);
    y_solution_out_port = new RTT::OutputPort<base::VectorXd>("y_solution_" + port_namespace);
    y_act_out_port = new RTT::OutputPort<base::VectorXd>("y_act_" + port_namespace);
    A_task_out_port = new RTT::OutputPort<base::MatrixXd>("task_mat_" + port_namespace);
}

SubTaskInterface::~SubTaskInterface()
{
    delete weight_port;
    delete activation_port;
    delete y_solution_out_port;
    delete y_act_out_port;
    delete A_task_out_port;
    if(pose_out_port)
        delete pose_out_port;
    if(cart_ref_port)
        delete cart_ref_port;
    if(jnt_ref_port)
        delete jnt_ref_port;
}

void SubTaskInterface::resetTimeout(){
    last_task_input = base::Time::now();
}

void SubTaskInterface::update(){

    // Read
    activation_port->read(activation);
    weight_port->read(weights);

    if(cart_ref_port){
        if(cart_ref_port->read(cart_ref) == RTT::NewData)
            last_task_input = base::Time::now();
    }
    else{
        if(jnt_ref_port->read(jnt_ref) == RTT::NewData)
            last_task_input = base::Time::now();
    }

    // Validate
    if(config.type == wbc::task_type_cartesian){
        if(!cart_ref.hasValidVelocity() ||
                !cart_ref.hasValidAngularVelocity()){
            LOG_ERROR("Reference input of task %s has invalid velocity and/or angular velocity", config.name.c_str());
            throw std::invalid_argument("Invalid Cartesian reference input");
        }

        y_des.segment(0,3) = cart_ref.velocity;
        y_des.segment(3,3) = cart_ref.angular_velocity;

        if(weights.size() !=6){
            LOG_ERROR("Input size for joint weights of task %s should be %i but is %i", config.name.c_str(), 6, weights.size());
            throw std::invalid_argument("Invalid weight input size");
        }
    }
    else{
        if(jnt_ref.size() != config.joint_names.size()){
            LOG_ERROR("Size for input reference of task %s should be %i but is %i", config.name.c_str(), config.joint_names.size(), jnt_ref.size());
            throw std::invalid_argument("Invalid joint reference input");
        }

        for(uint i = 0; i < config.joint_names.size(); i++){
            if(!jnt_ref[i].hasSpeed()){
                LOG_ERROR("Reference input for joint %s of task %s has invalid speed value(s)", jnt_ref.names[i].c_str(), config.name.c_str());
                throw std::invalid_argument("Invalid joint reference input");
            }
            y_des(i) = jnt_ref[i].speed;
        }

        if(weights.size() != config.joint_names.size()){
            LOG_ERROR("Input size for joint weights of task %s should be %i but is %i", config.name.c_str(), config.joint_names.size(), weights.size());
            throw std::invalid_argument("Invalid weight input size");
        }
    }
}
