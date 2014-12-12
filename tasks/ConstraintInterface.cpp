#include "ConstraintInterface.hpp"
#include <kdl_conversions/KDLConversions.hpp>
#include <wbc/ExtendedConstraint.hpp>

namespace wbc{

ConstraintInterface::ConstraintInterface(Constraint* _constraint)
{
    std::string port_namespace;
    constraint = _constraint;
    ConstraintConfig config = constraint->config;

    if(config.type == wbc::cart){
        pose_out_port = new RTT::OutputPort<base::samples::RigidBodyState>("pose_" + config.name);
        cart_ref_port = new RTT::InputPort<base::samples::RigidBodyState>("ref_" + config.name);
        jnt_ref_port = 0;
        joint_state_out_port = 0;
        kdl_conversions::KDL2RigidBodyState(KDL::Twist::Zero(), cart_ref);
    }
    else{
        jnt_ref_port = new RTT::InputPort<base::samples::Joints>("ref_" + config.name);
        joint_state_out_port = new RTT::OutputPort<base::samples::Joints>("joint_state_" + config.name);
        cart_ref_port = 0;
        pose_out_port = 0;
        jnt_ref.resize(config.joint_names.size());
        jnt_ref.names = config.joint_names;
        for(uint i = 0; i < config.joint_names.size(); i++)
            jnt_ref[i].speed = 0;
        constraint_jnt_state_.resize(config.joint_names.size());
        constraint_jnt_state_.names = config.joint_names;
    }

    activation_port = new RTT::InputPort<double>("activation_" + config.name);
    weight_port = new RTT::InputPort<base::VectorXd>("weight_" + config.name);
}

ConstraintInterface::~ConstraintInterface()
{
    delete weight_port;
    delete activation_port;
    if(pose_out_port)
        delete pose_out_port;
    if(cart_ref_port)
        delete cart_ref_port;
    if(jnt_ref_port)
        delete jnt_ref_port;
    if(joint_state_out_port)
        delete joint_state_out_port;
}

void ConstraintInterface::update(const base::samples::Joints& joint_state){

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

    if(constraint->config.type == wbc::cart)
    {
        kdl_conversions::KDL2RigidBodyState(((ExtendedConstraint*)constraint)->pose, constraint_pose_);
        constraint_pose_.time = base::Time::now();
        constraint_pose_.sourceFrame = constraint->config.tip;
        constraint_pose_.targetFrame = constraint->config.root;
        pose_out_port->write(constraint_pose_);
    }
    else
    {
        for(uint i = 0; i < constraint_jnt_state_.size(); i++)
        {
            uint idx;
            const std::string& joint_name = constraint_jnt_state_.names[i];
            try{
                idx = joint_state.mapNameToIndex(joint_name);
            }
            catch(std::exception e){
                LOG_ERROR("Constraint %s contains joint with name %s, but this name is not in joint state vector",
                          constraint->config.name.c_str(), joint_name.c_str());
                throw e;
            }
            constraint_jnt_state_[i] = joint_state[idx];
        }
        joint_state_out_port->write(constraint_jnt_state_);
    }
}

void ConstraintInterface::reset()
{
    constraint->reset();
}

void ConstraintInterface::addPortsToTaskContext(RTT::TaskContext* task)
{
    if(constraint->config.type == wbc::cart){
        task->ports()->addPort(cart_ref_port->getName(), *(cart_ref_port));
        task->ports()->addPort(pose_out_port->getName(), *(pose_out_port));
    }
    else{
        task->ports()->addPort((jnt_ref_port)->getName(), *(jnt_ref_port));
        task->ports()->addPort((joint_state_out_port)->getName(), *(joint_state_out_port));
    }

    task->ports()->addPort(weight_port->getName(), *(weight_port));
    task->ports()->addPort(activation_port->getName(), *(activation_port));
}

void ConstraintInterface::removePortsFromTaskContext(RTT::TaskContext* task)
{
    task->ports()->removePort(weight_port->getName());
    task->ports()->removePort(activation_port->getName());


    if(constraint->config.type == cart)
    {
        task->ports()->removePort(pose_out_port->getName());
        task->ports()->removePort(cart_ref_port->getName());
    }
    else{
        task->ports()->removePort(jnt_ref_port->getName());
        task->ports()->removePort(joint_state_out_port->getName());
    }

    task->ports()->removePort(weight_port->getName());
    task->ports()->removePort(activation_port->getName());
}
}
