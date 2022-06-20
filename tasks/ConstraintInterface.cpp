#include "ConstraintInterface.hpp"
#include <wbc/core/Scene.hpp>
#include <wbc/core/RobotModel.hpp>
#include <wbc/core/ConstraintStatus.hpp>

namespace wbc{

ConstraintInterface::ConstraintInterface(ConstraintConfig _cfg,
                                         WbcScenePtr _scene,
                                         RobotModelPtr _robot_model,
                                         RTT::TaskContext* _task_context){

    robot_model = _robot_model;
    task_context = _task_context;
    scene = _scene;

    cfg = _cfg;
    activation = cfg.activation;
    weights = base::VectorXd::Map(cfg.weights.data(), cfg.nVariables());

    if(cfg.type == cart || cfg.type == com){
        cart_ref_port = std::make_shared<CartRefPort>("ref_" + cfg.name);
        task_context->ports()->addPort(cart_ref_port->getName(), *(cart_ref_port));

        cart_state_out_port = std::make_shared<CartStatusPort>("status_" + cfg.name);
        task_context->ports()->addPort(cart_state_out_port->getName(), *(cart_state_out_port));

        cart_ref.twist.setZero();
        cart_ref.acceleration.setZero();
    }
    else{
        jnt_ref_port = std::make_shared<JntRefPort>("ref_" + cfg.name);
        task_context->ports()->addPort((jnt_ref_port)->getName(), *(jnt_ref_port));

        jnt_state_out_port = std::make_shared<JntStatusPort>("status_" + cfg.name);
        task_context->ports()->addPort((jnt_state_out_port)->getName(), *(jnt_state_out_port));

        jnt_ref.resize(cfg.joint_names.size());
        jnt_ref.names = cfg.joint_names;
        for(auto &e : jnt_ref.elements){
            e.speed = 0;
            e.acceleration = 0;
        }
        constraint_jnt_state.resize(cfg.joint_names.size());
        constraint_jnt_state.names = cfg.joint_names;
    }

    activation_port = std::make_shared<ActivationPort>("activation_" + cfg.name);
    task_context->ports()->addPort(activation_port->getName(), *(activation_port));

    weight_port = std::make_shared<WeightInPort>("weight_" + cfg.name);
    task_context->ports()->addPort(weight_port->getName(), *(weight_port));

    constraint_status_port = std::make_shared<ConstraintStatusPort>("constraint_" + cfg.name);
    task_context->ports()->addPort(constraint_status_port->getName(), *(constraint_status_port));
}

ConstraintInterface::~ConstraintInterface(){

    task_context->ports()->removePort(weight_port->getName());
    task_context->ports()->removePort(activation_port->getName());
    if(cart_state_out_port)
        task_context->ports()->removePort(cart_state_out_port->getName());
    if(cart_ref_port)
        task_context->ports()->removePort(cart_ref_port->getName());
    if(jnt_ref_port)
        task_context->ports()->removePort(jnt_ref_port->getName());
    if(jnt_state_out_port)
        task_context->ports()->removePort(jnt_state_out_port->getName());
    if(constraint_status_port)
        task_context->ports()->removePort(constraint_status_port->getName());
}

void ConstraintInterface::update(){

    if(activation_port->readNewest(activation) == RTT::NewData)
        scene->setTaskActivation(cfg.name,activation);
    if(weight_port->readNewest(weights) == RTT::NewData)
        scene->setTaskWeights(cfg.name,weights);

    if(cart_ref_port){
        if(cart_ref_port->readNewest(cart_ref) == RTT::NewData)
            scene->setReference(cfg.name,cart_ref);
    }
    else{
        if(jnt_ref_port->readNewest(jnt_ref) == RTT::NewData)
            scene->setReference(cfg.name,jnt_ref);
    }

    if(cfg.type == cart)
        cart_state_out_port->write(robot_model->rigidBodyState(cfg.ref_frame, cfg.tip));
    else if(cfg.type == com)
        cart_state_out_port->write(robot_model->rigidBodyState(robot_model->worldFrame(), robot_model->baseFrame()));
    else
        jnt_state_out_port->write(robot_model->jointState(cfg.joint_names));
}

void ConstraintInterface::writeConstraintStatus(const ConstraintStatus& status){
    constraint_status_port->write(status);
}

}
