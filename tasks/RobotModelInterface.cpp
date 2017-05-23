#include "RobotModelInterface.hpp"
#include <wbc/RobotModelConfig.hpp>

namespace wbc{

RobotModelInterface::RobotModelInterface(RTT::TaskContext* task){
    task_context = task;
}

RobotModelInterface::~RobotModelInterface(){
    for(it = pose_ports.begin(); it != pose_ports.end(); it++){
        task_context->ports()->removePort(it->second->getName());
        delete it->second;
    }
}

void RobotModelInterface::configure(const std::vector<RobotModelConfig> &config){

    for(size_t i = 0; i < config.size(); i++)
        if(!config[i].hook.empty())  // Don't create an input port if the hook is empty
            addPort(config[i].hook + "_pose");

    // Remove ports which are not required anymore. This is required
    // if wbc is re-configured with different constraints
    for(it = pose_ports.begin(); it != pose_ports.end(); it++){

        bool is_port_required = false;
        for(size_t j = 0; j < config.size(); j++){
            if(it->first == (config[j].hook + "_pose"))
                is_port_required = true;
        }
        if(!is_port_required){
            removePort(it->first);
            it--;
        }
    }
}

std::vector<base::samples::RigidBodyState> RobotModelInterface::update(){

    std::vector<base::samples::RigidBodyState> poses;
    base::samples::RigidBodyState pose;
    for(it = pose_ports.begin(); it != pose_ports.end(); it++){
        if(it->second->readNewest(pose) == RTT::NewData)
            poses.push_back(pose);
    }
    return poses;
}

void RobotModelInterface::addPort(const std::string port_name){

    if(!task_context->getPort(port_name)){ // Don't recreate ports
        PosePort* port = new PosePort(port_name);
        pose_ports[port_name] = port;
        task_context->ports()->addPort(port_name, *port);
    }
}

void RobotModelInterface::removePort(const std::string port_name){
    if(pose_ports.count(port_name) > 0){
        task_context->ports()->removePort(port_name);
        delete pose_ports[port_name];
        pose_ports.erase(port_name);
    }
}

}
