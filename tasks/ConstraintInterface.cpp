#include "ConstraintInterface.hpp"
#include <kdl_conversions/KDLConversions.hpp>
#include <wbc/common/Constraint.hpp>
#include <wbc/Wbc.hpp>
#include <wbc/robot_models/RobotModel.hpp>

namespace wbc{

ConstraintInterface::ConstraintInterface(const std::string &_constraint_name,
                                         Wbc* _wbc,
                                         RobotModel* _robot_model,
                                         RTT::TaskContext* _task_context){

    wbc = _wbc;
    robot_model = _robot_model;
    constraint_name = _constraint_name;
    task_context = _task_context;

    jnt_ref_port = 0;
    jnt_state_out_port = 0;
    cart_ref_port = 0;
    cart_state_out_port = 0;

    const ConstraintConfig& cfg = wbc->getConstraint(constraint_name)->config;

    if(cfg.type == wbc::cart){
        cart_ref_port = new RTT::InputPort<base::samples::RigidBodyState>("ref_" + constraint_name);
        task_context->ports()->addPort(cart_ref_port->getName(), *(cart_ref_port));

        cart_state_out_port = new RTT::OutputPort<base::samples::RigidBodyState>("pose_" + constraint_name);
        task_context->ports()->addPort(cart_state_out_port->getName(), *(cart_state_out_port));

        kdl_conversions::KDL2RigidBodyState(KDL::Twist::Zero(), cart_ref);
    }
    else{
        jnt_ref_port = new RTT::InputPort<base::samples::Joints>("ref_" + constraint_name);
        task_context->ports()->addPort((jnt_ref_port)->getName(), *(jnt_ref_port));

        jnt_state_out_port = new RTT::OutputPort<base::samples::Joints>("joint_state_" + constraint_name);
        task_context->ports()->addPort((jnt_state_out_port)->getName(), *(jnt_state_out_port));

        jnt_ref.resize(cfg.joint_names.size());
        jnt_ref.names = cfg.joint_names;
        for(uint i = 0; i < cfg.joint_names.size(); i++)
            jnt_ref[i].speed = 0;
        constraint_jnt_state.resize(cfg.joint_names.size());
        constraint_jnt_state.names = cfg.joint_names;
    }

    activation_port = new RTT::InputPort<double>("activation_" + constraint_name);
    task_context->ports()->addPort(activation_port->getName(), *(activation_port));

    weight_port = new RTT::InputPort<base::VectorXd>("weight_" + constraint_name);
    task_context->ports()->addPort(weight_port->getName(), *(weight_port));

    constraint_out_port = new RTT::OutputPort<wbc::Constraint>("constraint_" + constraint_name);
    task_context->ports()->addPort(constraint_out_port->getName(), *(constraint_out_port));
}

ConstraintInterface::~ConstraintInterface(){

    task_context->ports()->removePort(weight_port->getName());
    delete weight_port;

    task_context->ports()->removePort(activation_port->getName());
    delete activation_port;

    task_context->ports()->removePort(constraint_out_port->getName());
    delete constraint_out_port;

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
}

void ConstraintInterface::update(){

    if(activation_port->readNewest(activation) == RTT::NewData)
        wbc->setConstraintActivation(constraint_name, activation);
    if(weight_port->readNewest(weights) == RTT::NewData)
        wbc->setConstraintWeights(constraint_name, weights);

    if(cart_ref_port){
        if(cart_ref_port->readNewest(cart_ref) == RTT::NewData)
            wbc->setReference(constraint_name, cart_ref);
    }
    else{
        if(jnt_ref_port->readNewest(jnt_ref) == RTT::NewData)
            wbc->setReference(constraint_name, jnt_ref);
    }

    const ConstraintConfig& cfg = wbc->getConstraint(constraint_name)->config;
    if(cfg.type == cart){
        robot_model->getState(cfg.ref_frame, cfg.tip, constraint_cart_state);
        cart_state_out_port->write(constraint_cart_state);
    }
    else{
        robot_model->getState(cfg.joint_names, constraint_jnt_state);
        jnt_state_out_port->write(constraint_jnt_state);
    }

    constraint_out_port->write(*wbc->getConstraint(constraint_name));
}

void ConstraintInterface::reset(){
    wbc->getConstraint(constraint_name)->reset();
}
}
