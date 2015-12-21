#include "ConstraintInterface.hpp"
#include <kdl_conversions/KDLConversions.hpp>
#include <wbc/ExtendedConstraint.hpp>

namespace wbc{

ConstraintInterface::ConstraintInterface(Constraint* _constraint, RTT::TaskContext* task)
{
    constraint = _constraint;
    const ConstraintConfig &config = constraint->config;
    task_context = task;

    if(config.type == wbc::cart){
        cart_ref_port = new RTT::InputPort<base::samples::RigidBodyState>("ref_" + config.name);
        task_context->ports()->addPort(cart_ref_port->getName(), *(cart_ref_port));

        pose_out_port = new RTT::OutputPort<base::samples::RigidBodyState>("pose_" + config.name);
        task_context->ports()->addPort(pose_out_port->getName(), *(pose_out_port));

        jnt_ref_port = 0;
        joint_state_out_port = 0;
        kdl_conversions::KDL2RigidBodyState(KDL::Twist::Zero(), cart_ref);
    }
    else{
        jnt_ref_port = new RTT::InputPort<base::samples::Joints>("ref_" + config.name);
        task_context->ports()->addPort((jnt_ref_port)->getName(), *(jnt_ref_port));

        joint_state_out_port = new RTT::OutputPort<base::samples::Joints>("joint_state_" + config.name);
        task_context->ports()->addPort((joint_state_out_port)->getName(), *(joint_state_out_port));

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
    task_context->ports()->addPort(activation_port->getName(), *(activation_port));

    weight_port = new RTT::InputPort<base::VectorXd>("weight_" + config.name);
    task_context->ports()->addPort(weight_port->getName(), *(weight_port));

}

ConstraintInterface::~ConstraintInterface()
{
    task_context->ports()->removePort(weight_port->getName());
    delete weight_port;

    task_context->ports()->removePort(activation_port->getName());
    delete activation_port;


    if(constraint->config.type == cart)
    {
        task_context->ports()->removePort(pose_out_port->getName());
        delete pose_out_port;

        task_context->ports()->removePort(cart_ref_port->getName());
        delete cart_ref_port;
    }
    else{
        task_context->ports()->removePort(jnt_ref_port->getName());
        delete jnt_ref_port;

        task_context->ports()->removePort(joint_state_out_port->getName());
        delete joint_state_out_port;
    }
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
        kdl_conversions::KDL2RigidBodyState(((ExtendedConstraint*)constraint)->pose_ref_frame_in_root, constraint_pose_);
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
}
