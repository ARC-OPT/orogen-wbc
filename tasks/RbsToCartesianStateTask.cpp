/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RbsToCartesianStateTask.hpp"

using namespace wbc;

RbsToCartesianStateTask::RbsToCartesianStateTask(std::string const& name)
    : RbsToCartesianStateTaskBase(name){
}

RbsToCartesianStateTask::~RbsToCartesianStateTask(){
}

bool RbsToCartesianStateTask::configureHook(){
    if (! RbsToCartesianStateTaskBase::configureHook())
        return false;

    std::vector<std::string> input_ports = _input_ports.get();

    for(auto in : input_ports){
        RbsInputPortPtr in_port = std::make_shared<RbsInputPort>(in);
        this->ports()->addEventPort(in_port->getName(), *in_port);
        input_port_map[in] = in_port;

        CartesianStateOutputPortPtr out_port = std::make_shared<CartesianStateOutputPort>(in + "_out");
        this->ports()->addPort(out_port->getName(), *out_port);
        output_port_map[in] = out_port;
    }

    return true;
}

bool RbsToCartesianStateTask::startHook(){
    if (! RbsToCartesianStateTaskBase::startHook())
        return false;
    return true;
}

void RbsToCartesianStateTask::updateHook(){
    RbsToCartesianStateTaskBase::updateHook();

    base::samples::RigidBodyState rbs_in;
    for(auto in : input_port_map){
        while(in.second->read(rbs_in) == RTT::NewData)
            output_port_map[in.first]->write(fromRigidBodyState(rbs_in));
    }
}

void RbsToCartesianStateTask::errorHook(){
    RbsToCartesianStateTaskBase::errorHook();
}

void RbsToCartesianStateTask::stopHook(){
    RbsToCartesianStateTaskBase::stopHook();
}

void RbsToCartesianStateTask::cleanupHook(){
    RbsToCartesianStateTaskBase::cleanupHook();

    for(auto in : input_port_map)
        this->ports()->removePort(in.second->getName());

    for(auto out : output_port_map)
        this->ports()->removePort(out.second->getName());

    input_port_map.clear();
    output_port_map.clear();
}

base::samples::RigidBodyStateSE3 RbsToCartesianStateTask::fromRigidBodyState(const base::samples::RigidBodyState& in){
    base::samples::RigidBodyStateSE3 out;
    out.time = in.time;
    out.frame_id = in.targetFrame;
    out.pose.position = in.position;
    out.pose.orientation = in.orientation;
    out.twist.linear = in.velocity;
    out.twist.angular = in.angular_velocity;
    return out;
}
