#include "RobotModelInterface.hpp"
#include <wbc/RobotModelConfig.hpp>

namespace wbc{

RobotModelInterface::RobotModelInterface(RTT::TaskContext* task){
    task_context = task;
}

RobotModelInterface::~RobotModelInterface(){
    pose_in_ports.clear();
    pose_out_ports.clear();
}

void RobotModelInterface::configure(const std::vector<std::string> &names){

    for(const std::string& n : names){
        addInputPort(n + "_pose");
        addOutputPort("current_" + n + "_pose");
    }

    // Remove ports which are not required anymore. This is required
    // if wbc is re-configured with different constraints
    for(auto it = pose_in_ports.begin(); it != pose_in_ports.end();){

        bool is_port_required = false;
        for(const std::string &n : names){
            if(it->first == (n + "_pose"))
                is_port_required = true;
        }
        if(is_port_required)
             it++;
        else
            removeInputPort(it->first);
    }
    for(auto oit = pose_out_ports.begin(); oit != pose_out_ports.end();){

        bool is_port_required = false;
        for(const std::string &n : names){
            if(oit->first == ("current_" + n + "_pose"))
                is_port_required = true;
        }
        if(is_port_required)
             oit++;
        else
            removeOutputPort(oit->first);
    }
}

std::vector<CartesianState> RobotModelInterface::update(){

    std::vector<CartesianState> poses;
    for(const auto& it : pose_in_ports){
        if(it.second->readNewest(model_pose) == RTT::NewData){
            poses.push_back(model_pose);
            pose_out_ports["current_" + it.first]->write(model_pose);
        }
    }
    return poses;
}

void RobotModelInterface::addInputPort(const std::string port_name){

    if(!task_context->getPort(port_name)){ // Don't recreate ports
        PoseInPortPtr port = std::make_shared<PoseInPort>();
        pose_in_ports[port_name] = port;
        task_context->ports()->addPort(port_name, *port);
    }
}

void RobotModelInterface::removeInputPort(const std::string port_name){
    if(pose_in_ports.count(port_name) > 0){
        task_context->ports()->removePort(port_name);
        pose_in_ports.erase(port_name);
    }
}

void RobotModelInterface::addOutputPort(const std::string port_name){

    if(!task_context->getPort(port_name)){ // Don't recreate ports
        PoseOutPortPtr port = std::make_shared<PoseOutPort>();
        pose_out_ports[port_name] = port;
        task_context->ports()->addPort(port_name, *port);
    }
}

void RobotModelInterface::removeOutputPort(const std::string port_name){
    if(pose_out_ports.count(port_name) > 0){
        task_context->ports()->removePort(port_name);
        pose_out_ports.erase(port_name);
    }
}

}
