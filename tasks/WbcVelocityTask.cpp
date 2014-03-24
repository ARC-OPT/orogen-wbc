/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include <wbc/SubTask.hpp>
#include "WbcVelocityTask.hpp"
#include <wbc/TaskFrame.hpp>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <wbc/ExtendedSubTask.hpp>
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
    if(!wbc_.configure(tree, wbc_config, _joint_names.get(), _tasks_active.get(), _task_timeout.get()))
        return false;
    wbc_.solver()->setNormMax(_norm_max.get());
    wbc_.solver()->setSVDMethod(_svd_method.get());
    joint_weights_ = _initial_joint_weights.get();
    if(joint_weights_.size() != 0)
    {
        joint_weight_mat_ = base::MatrixXd(joint_weights_.size(), joint_weights_.size());
        joint_weight_mat_.setIdentity();
        joint_weight_mat_.diagonal() = joint_weights_;
        wbc_.solver()->setJointWeights(joint_weight_mat_);
    }

    LOG_DEBUG("Configuring WBC Config done");

    //
    // Create ports
    //
    for(uint i = 0; i < wbc_config.size(); i++)
    {
        SubTask* sub_task = wbc_.subTask(wbc_config[i].name);
        SubTaskInterface* sti = new SubTaskInterface(sub_task);

        addPortsForSubTask(sti);
        sub_task_interface_map_[wbc_config[i].name] = sti;
    }

    LOG_DEBUG("Created ports");

    solver_output_.resize(wbc_.noOfJoints());
    solver_output_.setZero();
    act_robot_velocity_.resize(wbc_.noOfJoints());
    act_robot_velocity_.setZero();
    joint_weights_ = base::VectorXd::Ones(wbc_.noOfJoints());
    joint_weight_mat_.resize(wbc_.noOfJoints(), wbc_.noOfJoints());
    joint_weight_mat_.setIdentity();
    ctrl_out_.resize(wbc_.noOfJoints());
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
        wbc_.solver()->setJointWeights((Eigen::MatrixXd& )joint_weight_mat_);
    }

    //
    // Compute control solution
    //
    wbc_.solve(joint_status_, (Eigen::VectorXd& )solver_output_);

    //
    // Write output
    //
    if(ctrl_out_.empty()){
        ctrl_out_.resize(joint_status_.size());
        ctrl_out_.names = joint_status_.names;
    }
    for(uint i = 0; i < ctrl_out_.size(); i++){
        uint idx = wbc_.jointIndex(ctrl_out_.names[i]);
        ctrl_out_[i].speed = solver_output_(idx);
    }
    ctrl_out_.time = base::Time::now();
    _ctrl_out.write(ctrl_out_);

    //
    // write Debug Data
    //
    _damping.write(wbc_.solver()->getCurDamping());
    for(SubTaskInterfaceMap::iterator it = sub_task_interface_map_.begin(); it != sub_task_interface_map_.end(); it++)
    {
        SubTask* task = wbc_.subTask(it->first);
        SubTaskInterface *iface = it->second;
        if(task->config.type == wbc::task_type_cartesian)
        {
            base::samples::RigidBodyState rbs;
            kdl_conversions::KDL2RigidBodyState(((ExtendedSubTask*)task)->pose, rbs);
            rbs.time = base::Time::now();
            rbs.sourceFrame = task->config.root;
            rbs.targetFrame = task->config.tip;

            iface->pose_out_port->write(rbs);

            if((base::Time::now() - stamp_).toSeconds() > 3){
                LOG_DEBUG_S<<"TF: ", task->config.name.c_str();
                LOG_DEBUG_S<<"SourceFrame: "<<rbs.sourceFrame<<", TargetFrame: "<<rbs.targetFrame;
                LOG_DEBUG_S<<"Position: "<<rbs.position;
                LOG_DEBUG_S<<"Ori Euler XYZ: "<<base::getEuler(rbs.orientation)<<endl;
            }
        }
    }
    if((base::Time::now() - stamp_).toSeconds() > 3)
        stamp_ = base::Time::now();
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

            task->y_solution = task->A * solver_output_;
            task->y = task->A * act_robot_velocity_;
            iface->sub_task_out_port->write(*task);
        }
    }

    _sample_time.write((base::Time::now() - start).toSeconds());
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


SubTaskInterface::SubTaskInterface(SubTask* _sub_task)
{
    std::string port_namespace;
    uint no_task_vars;
    sub_task = _sub_task;
    SubTaskConfig config = sub_task->config;

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

    activation_port = new RTT::InputPort<double>("activation_" + port_namespace);
    weight_port = new RTT::InputPort<base::VectorXd>("weight_" + port_namespace);
    sub_task_out_port = new RTT::OutputPort<SubTask>("sub_task_" + port_namespace);
}

SubTaskInterface::~SubTaskInterface()
{
    delete weight_port;
    delete activation_port;
    delete sub_task_out_port;
    if(pose_out_port)
        delete pose_out_port;
    if(cart_ref_port)
        delete cart_ref_port;
    if(jnt_ref_port)
        delete jnt_ref_port;
}

void SubTaskInterface::update(){

    // Read
    activation_port->read(sub_task->activation);
    weight_port->read(sub_task->weights);

    if(cart_ref_port){
        if(cart_ref_port->read(cart_ref) == RTT::NewData){
            sub_task->last_task_input = base::Time::now();
        }
    }
    else{
        if(jnt_ref_port->read(jnt_ref) == RTT::NewData)
            sub_task->last_task_input = base::Time::now();
    }

    // Validate
    if(sub_task->config.type == wbc::task_type_cartesian){
        if(!cart_ref.hasValidVelocity() ||
                !cart_ref.hasValidAngularVelocity()){
            LOG_ERROR("Reference input of task %s has invalid velocity and/or angular velocity", sub_task->config.name.c_str());
            throw std::invalid_argument("Invalid Cartesian reference input");
        }

        sub_task->y_des.segment(0,3) = cart_ref.velocity;
        sub_task->y_des.segment(3,3) = cart_ref.angular_velocity;
    }
    else{
        if(jnt_ref.size() != sub_task->no_task_vars){
            LOG_ERROR("Size for input reference of task %s should be %i but is %i", sub_task->config.name.c_str(), sub_task->no_task_vars, jnt_ref.size());
            throw std::invalid_argument("Invalid joint reference input");
        }

        for(uint i = 0; i < sub_task->config.joint_names.size(); i++){
            if(!jnt_ref[i].hasSpeed()){
                LOG_ERROR("Reference input for joint %s of task %s has invalid speed value(s)", jnt_ref.names[i].c_str(), sub_task->config.name.c_str());
                throw std::invalid_argument("Invalid joint reference input");
            }
            sub_task->y_des(i) = jnt_ref[i].speed;
        }
    }

    if(sub_task->activation < 0 || sub_task->activation > 1){
        LOG_ERROR("Sub Task: %s. Activation must be >= 0 and <= 1, but is %f", sub_task->config.name.c_str(), sub_task->activation);
        throw std::invalid_argument("Invalid activation value");
    }
    if(sub_task->weights.size() !=  sub_task->no_task_vars){
        LOG_ERROR("Size of weight vector is %f, but task %s has %f task variables", sub_task->weights.size(), sub_task->config.name.c_str(), sub_task->no_task_vars);
        throw std::invalid_argument("Invalid no of task weights");
    }
}
