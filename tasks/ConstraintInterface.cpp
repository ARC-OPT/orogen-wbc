#include "ConstraintInterface.hpp"
#include <kdl_conversions/KDLConversions.hpp>
#include <wbc/CartesianConstraint.hpp>
#include <wbc/JointConstraint.hpp>
#include <wbc/RobotModel.hpp>

namespace wbc{

ConstraintInterface::ConstraintInterface(ConstraintPtr _constraint,
                                         RobotModelPtr _robot_model,
                                         RTT::TaskContext* _task_context){

    constraint = _constraint;
    robot_model = _robot_model;
    task_context = _task_context;

    jnt_ref_port = 0;
    jnt_state_out_port = 0;
    cart_ref_port = 0;
    cart_state_out_port = 0;

    const ConstraintConfig &cfg = constraint->config;

    if(constraint->config.type == cart){
        cart_ref_port = new RTT::InputPort<base::samples::RigidBodyState>("ref_" + cfg.name);
        task_context->ports()->addPort(cart_ref_port->getName(), *(cart_ref_port));

        cart_state_out_port = new RTT::OutputPort<base::samples::RigidBodyState>("pose_" + cfg.name);
        task_context->ports()->addPort(cart_state_out_port->getName(), *(cart_state_out_port));

        kdl_conversions::KDL2RigidBodyState(KDL::Twist::Zero(), cart_ref);
    }
    else{
        jnt_ref_port = new RTT::InputPort<base::samples::Joints>("ref_" + cfg.name);
        task_context->ports()->addPort((jnt_ref_port)->getName(), *(jnt_ref_port));

        jnt_state_out_port = new RTT::OutputPort<base::samples::Joints>("joint_state_" + cfg.name);
        task_context->ports()->addPort((jnt_state_out_port)->getName(), *(jnt_state_out_port));

        jnt_ref.resize(cfg.joint_names.size());
        jnt_ref.names = cfg.joint_names;
        for(uint i = 0; i < cfg.joint_names.size(); i++)
            jnt_ref[i].speed = 0;
        constraint_jnt_state.resize(cfg.joint_names.size());
        constraint_jnt_state.names = cfg.joint_names;
    }

    activation_port = new RTT::InputPort<double>("activation_" + cfg.name);
    task_context->ports()->addPort(activation_port->getName(), *(activation_port));

    weight_port = new RTT::InputPort<base::VectorXd>("weight_" + cfg.name);
    task_context->ports()->addPort(weight_port->getName(), *(weight_port));

    constraint_out_port = new RTT::OutputPort<wbc::Constraint>("constraint_" + cfg.name);
    task_context->ports()->addPort(constraint_out_port->getName(), *(constraint_out_port));
}

ConstraintInterface::~ConstraintInterface(){

    task_context->ports()->removePort(weight_port->getName());
    delete weight_port;

    task_context->ports()->removePort(activation_port->getName());
    delete activation_port;

    if(cart_state_out_port){
        task_context->ports()->removePort(cart_state_out_port->getName());
        delete cart_state_out_port;
    }
    if(cart_ref_port){
        task_context->ports()->removePort(cart_ref_port->getName());
        delete cart_ref_port;
    }
    if(jnt_ref_port){
        task_context->ports()->removePort(jnt_ref_port->getName());
        delete jnt_ref_port;
    }
    if(jnt_state_out_port){
        task_context->ports()->removePort(jnt_state_out_port->getName());
        delete jnt_state_out_port;
    }
    if(constraint_out_port){
        task_context->ports()->removePort(constraint_out_port->getName());
        delete constraint_out_port;
    }
}

void ConstraintInterface::update(){

    if(activation_port->readNewest(activation) == RTT::NewData)
        constraint->setActivation(activation);
    if(weight_port->readNewest(weights) == RTT::NewData)
        constraint->setWeights(weights);

    if(cart_ref_port){
        if(cart_ref_port->readNewest(cart_ref) == RTT::NewData){
            std::shared_ptr<CartesianConstraint> ptr = std::static_pointer_cast<CartesianConstraint>(constraint);
            ptr->setReference(cart_ref);
        }
    }
    else{
        if(jnt_ref_port->readNewest(jnt_ref) == RTT::NewData){
            std::shared_ptr<JointConstraint> ptr = std::static_pointer_cast<JointConstraint>(constraint);
            ptr->setReference(jnt_ref);
        }
    }

    const ConstraintConfig& cfg = constraint->config;
    if(cfg.type == cart)
        cart_state_out_port->write(robot_model->rigidBodyState(cfg.ref_frame, cfg.tip));
    else
        jnt_state_out_port->write(robot_model->jointState(cfg.joint_names));
}

void ConstraintInterface::reset(){
    constraint->reset();
}

void ConstraintInterface::writeDebug(){
    constraint_out_port->write(*constraint);
}

}
