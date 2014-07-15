/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include <wbc/Constraint.hpp>
#include "WbcVelocityTask.hpp"
#include <wbc/TaskFrame.hpp>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <wbc/ExtendedConstraint.hpp>
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

    std::vector<wbc::ConstraintConfig> wbc_config = _wbc_config.get();
    debug_ = _debug.get();

    if(!wbc_.configure(wbc_config, _joint_names.get(), _tasks_active.get(), _task_timeout.get()))
        return false;

    LOG_DEBUG("Configuring WBC Config done");
    
    //
    // Create ports
    //
    for(uint i = 0; i < wbc_config.size(); i++)
    {
        Constraint* constraint = wbc_.constraint(wbc_config[i].name);
        ConstraintInterface* sti = new ConstraintInterface(constraint);
        
        addPortsForConstraint(sti);
        constraint_interface_map_[wbc_config[i].name] = sti;
    }
    
    LOG_DEBUG("Created ports");

    act_robot_velocity_.resize(wbc_.noOfJoints());
    act_robot_velocity_.setZero();
    solver_output_eigen_.resize(wbc_.noOfJoints());
    solver_output_eigen_.setZero();
    
    return true;
}

bool WbcVelocityTask::startHook(){
    if (! WbcVelocityTaskBase::startHook())
        return false;

    //Clear all task references, weights etc. to have to secure initial state
    for(ConstraintInterfaceMap::iterator it = constraint_interface_map_.begin(); it != constraint_interface_map_.end(); it++)
        it->second->reset();

    return true;
}

void WbcVelocityTask::updateHook(){
    WbcVelocityTaskBase::updateHook();

    if(!stamp_.isNull())
        _sample_time.write((base::Time::now() - stamp_).toSeconds());
    stamp_ = base::Time::now();
    
    //
    // Read inputs
    //
    if(_task_frames.read(task_frames_) == RTT::NoData){
        LOG_DEBUG("No data on task frame port");
        return;
    }

    for(ConstraintInterfaceMap::iterator it = constraint_interface_map_.begin(); it != constraint_interface_map_.end(); it++)
        it->second->update();
    //
    // Compute control solution
    //
    wbc_.prepareEqSystem(task_frames_, solver_input_);
    
    //
    // Write output
    //
    _solver_input.write(solver_input_);

    for(ConstraintInterfaceMap::iterator it = constraint_interface_map_.begin(); it != constraint_interface_map_.end(); it++)
    {
        //TODO: This should be done somewhere else (tasks should get current poses from transformer!?)
        Constraint* constraint = wbc_.constraint(it->first);
        ConstraintInterface *iface = it->second;
        if(constraint->config.type == wbc::cart)
        {
            kdl_conversions::KDL2RigidBodyState(((ExtendedConstraint*)constraint)->pose, constraint_pose_);
            constraint_pose_.time = base::Time::now();
            constraint_pose_.sourceFrame = constraint->config.tip;
            constraint_pose_.targetFrame = constraint->config.root;
            iface->pose_out_port->write(constraint_pose_);
        }
    }

    //
    // write Debug Data
    //
    if(debug_)
    {
        if(_joint_state.read(joint_state_) == RTT::NoData)
            throw std::runtime_error("Current joint state is required to compute debug data, but there is no data on joint state port");
        if(_solver_output.read(solver_output_) == RTT::NoData)
            throw std::runtime_error("Solver Output is required to compute debug data, but there is no data on solver_output port");

        for(ConstraintInterfaceMap::iterator it = constraint_interface_map_.begin(); it != constraint_interface_map_.end(); it++)
        {
            Constraint* constraint = wbc_.constraint(it->first);
            ConstraintInterface *iface = it->second;
            for(size_t i = 0; i < joint_names_.size(); i++)
            {
                size_t idx = joint_state_.mapNameToIndex(joint_names_[i]);
                act_robot_velocity_(i) = joint_state_[idx].speed;
                solver_output_eigen_(i) = solver_output_[idx].speed;
            }

            constraint->computeDebug(solver_output_eigen_, act_robot_velocity_);
            iface->constraint_out_port->write(*constraint);
        }
    }

}

void WbcVelocityTask::cleanupHook()
{
    WbcVelocityTaskBase::cleanupHook();
    
    for(ConstraintInterfaceMap::iterator it = constraint_interface_map_.begin(); it != constraint_interface_map_.end(); it++)
    {
        removePortsOfConstraint(it->second);
        delete it->second;
    }
    constraint_interface_map_.clear();
}

void WbcVelocityTask::addPortsForConstraint(const ConstraintInterface* sti){
    if(sti->constraint->config.type == wbc::cart){
        ports()->addPort(sti->cart_ref_port->getName(), *(sti->cart_ref_port));
        ports()->addPort(sti->pose_out_port->getName(), *(sti->pose_out_port));
    }
    else
        ports()->addPort((sti->jnt_ref_port)->getName(), *(sti->jnt_ref_port));

    ports()->addPort(sti->weight_port->getName(), *(sti->weight_port));
    ports()->addPort(sti->activation_port->getName(), *(sti->activation_port));

    if(debug_)
        ports()->addPort(sti->constraint_out_port->getName(), *(sti->constraint_out_port));
}

void WbcVelocityTask::removePortsOfConstraint(const ConstraintInterface* sti)
{
    ports()->removePort(sti->weight_port->getName());
    ports()->removePort(sti->activation_port->getName());

    if(debug_)
    {
        if(ports()->getPort(sti->constraint_out_port->getName())){
            ports()->removePort(sti->constraint_out_port->getName());
        }
    }
    if(sti->constraint->config.type == cart)
    {
        if(ports()->getPort(sti->pose_out_port->getName())){
            ports()->removePort(sti->pose_out_port->getName());
        }
        if(ports()->getPort(sti->cart_ref_port->getName())){
            ports()->removePort(sti->cart_ref_port->getName());
        }
    }
    else{
        if(ports()->getPort(sti->jnt_ref_port->getName())){
            ports()->removePort(sti->jnt_ref_port->getName());
        }
    }
}

ConstraintInterface::ConstraintInterface(Constraint* _constraint)
{
    std::string port_namespace;
    constraint = _constraint;
    ConstraintConfig config = constraint->config;
    
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
        jnt_ref.resize(config.joint_names.size());
        jnt_ref.names = config.joint_names;
        for(uint i = 0; i < config.joint_names.size(); i++)
            jnt_ref[i].speed = 0;
    }
    
    activation_port = new RTT::InputPort<double>("activation_" + port_namespace);
    weight_port = new RTT::InputPort<base::VectorXd>("weight_" + port_namespace);
    constraint_out_port = new RTT::OutputPort<Constraint>("constraint_" + port_namespace);
}

ConstraintInterface::~ConstraintInterface()
{
    delete weight_port;
    delete activation_port;
    delete constraint_out_port;
    if(pose_out_port)
        delete pose_out_port;
    if(cart_ref_port)
        delete cart_ref_port;
    if(jnt_ref_port)
        delete jnt_ref_port;
}

void ConstraintInterface::update(){
    
    // Read
    activation_port->read(constraint->activation);
    weight_port->read(constraint->weights);
    
    if(cart_ref_port){
        if(cart_ref_port->read(cart_ref) == RTT::NewData)
            constraint->setReference(cart_ref);
    }
    else{
        if(jnt_ref_port->read(jnt_ref) == RTT::NewData)
            constraint->setReference(jnt_ref);
    }
    constraint->validate();
}

void ConstraintInterface::reset()
{
    constraint->reset();
}
