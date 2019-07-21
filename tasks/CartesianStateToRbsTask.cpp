/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianStateToRbsTask.hpp"

using namespace wbc;

CartesianStateToRbsTask::CartesianStateToRbsTask(std::string const& name)
    : CartesianStateToRbsTaskBase(name){
}

CartesianStateToRbsTask::~CartesianStateToRbsTask(){
}

bool CartesianStateToRbsTask::configureHook(){
    if (! CartesianStateToRbsTaskBase::configureHook())
        return false;

    std::vector<std::string> input_ports = _input_ports.get();

    for(auto in : input_ports){
        CartesianStateInputPortPtr in_port = std::make_shared<CartesianStateInputPort>(in);
        this->ports()->addEventPort(in_port->getName(), *in_port);
        input_port_map[in] = in_port;

        RbsOutputPortPtr out_port = std::make_shared<RbsOutputPort>(in + "_out");
        this->ports()->addPort(out_port->getName(), *out_port);
        output_port_map[in] = out_port;
    }

    return true;
}

bool CartesianStateToRbsTask::startHook(){
    if (! CartesianStateToRbsTaskBase::startHook())
        return false;
    return true;
}

void CartesianStateToRbsTask::updateHook(){
    CartesianStateToRbsTaskBase::updateHook();

    base::samples::CartesianState cartesian_state_in;
    for(auto in : input_port_map){
        while(in.second->read(cartesian_state_in) == RTT::NewData)
            output_port_map[in.first]->write(cartesianStateToRbs(cartesian_state_in));
    }
}

void CartesianStateToRbsTask::errorHook(){
    CartesianStateToRbsTaskBase::errorHook();
}

void CartesianStateToRbsTask::stopHook(){
    CartesianStateToRbsTaskBase::stopHook();
}

void CartesianStateToRbsTask::cleanupHook(){
    CartesianStateToRbsTaskBase::cleanupHook();

    for(auto in : input_port_map)
        this->ports()->removePort(in.first);

    for(auto out : output_port_map)
        this->ports()->removePort(out.first);

    input_port_map.clear();
    output_port_map.clear();
}

base::samples::RigidBodyState CartesianStateToRbsTask::cartesianStateToRbs(const base::samples::CartesianState& in){
    base::samples::RigidBodyState out;
    out.time = in.time;
    out.sourceFrame = in.source_frame;
    out.targetFrame = in.target_frame;
    out.position = in.pose.position;
    out.orientation = in.pose.orientation;
    out.velocity = in.twist.linear;
    out.angular_velocity = in.twist.angular;
    return out;
}
