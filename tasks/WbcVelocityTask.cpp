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
    debug_ = _debug.get();
    
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

    if(!wbc_.configure(tree, wbc_config, _joint_names.get(), _tasks_active.get(), _task_timeout.get(), _debug.get()))
        return false;

    wbc_.solver()->setNormMax(_norm_max.get());
    wbc_.solver()->setSVDMethod(_svd_method.get());
    wbc_.solver()->setComputeDebug(_debug.get());
    joint_weights_ = _initial_joint_weights.get();
    wbc_.solver()->setJointWeights(joint_weights_);
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
    for(uint i = 0; i < wbc_.solver()->getNoPriorities(); i++)
    {
        std::stringstream s;
        s <<  "prio_data_p" << i;
        RTT::OutputPort<PriorityData> *port = new RTT::OutputPort<PriorityData>(s.str());
        prio_data_ports_.push_back(port);
        ports()->addPort(port->getName(), *port);
    }

    
    LOG_DEBUG("Created ports");
    
    solver_output_.resize(wbc_.noOfJoints());
    solver_output_.setZero();
    act_robot_velocity_.resize(wbc_.noOfJoints());
    act_robot_velocity_.setZero();
    ctrl_out_.resize(wbc_.noOfJoints());
    ctrl_out_.names = wbc_.jointNames();
    
    return true;
}

bool WbcVelocityTask::startHook(){
    if (! WbcVelocityTaskBase::startHook())
        return false;

    //Clear all task references, weights etc. to have to secure initial state
    for(SubTaskInterfaceMap::iterator it = sub_task_interface_map_.begin(); it != sub_task_interface_map_.end(); it++)
        it->second->resetTask();

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
        wbc_.solver()->setJointWeights((Eigen::VectorXd& )joint_weights_);
    _current_joint_weights.write(joint_weights_);
    
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
    for(SubTaskInterfaceMap::iterator it = sub_task_interface_map_.begin(); it != sub_task_interface_map_.end(); it++)
    {
        //TODO: This should be done somewhere else (tasks should get current poses from transformer!?)
        SubTask* task = wbc_.subTask(it->first);
        SubTaskInterface *iface = it->second;
        if(task->config.type == wbc::cart)
        {
            base::samples::RigidBodyState rbs;
            kdl_conversions::KDL2RigidBodyState(((ExtendedSubTask*)task)->pose, rbs);
            rbs.time = base::Time::now();
            rbs.sourceFrame = task->config.tip;
            rbs.targetFrame = task->config.root;
            
            iface->pose_out_port->write(rbs);
        }
    }
    if(debug_)
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
            
            task->computeDebug(solver_output_, act_robot_velocity_);
            iface->sub_task_out_port->write(*task);
        }

        wbc_.solver()->getPrioDebugData(prio_data_);
        for(uint i = 0; i < prio_data_.size(); i++)
            prio_data_ports_[i]->write(prio_data_[i]);

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
    for(uint i = 0; i < prio_data_ports_.size(); i++)
    {
        ports()->removePort(prio_data_ports_[i]->getName());
        delete prio_data_ports_[i];
    }
    sub_task_interface_map_.clear();
    prio_data_ports_.clear();
}

SubTaskInterface::SubTaskInterface(SubTask* _sub_task)
{
    std::string port_namespace;
    sub_task = _sub_task;
    SubTaskConfig config = sub_task->config;
    
    if(config.type == wbc::cart){
        std::stringstream ss;
        ss<<"p"<<config.priority<<"_cart_"<<config.name;
        port_namespace = ss.str();
        
        pose_out_port = new RTT::OutputPort<base::samples::RigidBodyState>("pose_" + port_namespace);
        cart_ref_port = new RTT::InputPort<base::samples::RigidBodyState>("ref_" + port_namespace);
        jnt_ref_port = 0;
        kdl_conversions::KDL2RigidBodyState(KDL::Twist::Zero(), cart_ref);
    }
    else{
        std::stringstream ss;
        ss<<"p"<<config.priority<<"_jnt_"<<config.name;
        port_namespace = ss.str();
        
        jnt_ref_port = new RTT::InputPort<base::samples::Joints>("ref_" + port_namespace);
        cart_ref_port = 0;
        pose_out_port = 0;
        jnt_ref.resize(config.task_var_names.size());
        jnt_ref.names = config.task_var_names;
        for(uint i = 0; i < config.task_var_names.size(); i++)
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
        if(cart_ref_port->read(cart_ref) == RTT::NewData)
            sub_task->setReference(cart_ref);
    }
    else{
        if(jnt_ref_port->read(jnt_ref) == RTT::NewData)
            sub_task->setReference(jnt_ref);
    }
    sub_task->validate();
}

void SubTaskInterface::resetTask()
{
    sub_task->reset();
}
