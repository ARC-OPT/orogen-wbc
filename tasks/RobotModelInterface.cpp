#include "RobotModelInterface.hpp"
#include <wbc/RobotModelConfig.hpp>

namespace wbc{

RobotModelInterface::RobotModelInterface(RTT::TaskContext* task){
    task_context = task;
}

RobotModelInterface::~RobotModelInterface(){
    PoseInPortMap::iterator it;
    for(it = pose_in_ports.begin(); it != pose_in_ports.end(); it++){
        task_context->ports()->removePort(it->second->getName());
        delete it->second;
    }
    model_poses.clear();
}

void RobotModelInterface::configure(const std::vector<RobotModelConfig> &config){

    for(size_t i = 0; i < config.size(); i++)
        if(!config[i].hook.empty())  // Don't create an input port if the hook is empty
            addInputPort(config[i].initial_pose.sourceFrame + "_pose");

    // Remove ports which are not required anymore. This is required
    // if wbc is re-configured with different constraints
    PoseInPortMap::iterator it;
    for(it = pose_in_ports.begin(); it != pose_in_ports.end(); it++){

        bool is_port_required = false;
        for(size_t j = 0; j < config.size(); j++){
            if(it->first == (config[j].initial_pose.sourceFrame + "_pose"))
                is_port_required = true;
        }
        if(!is_port_required){
            removeInputPort(it->first);
            it--;
        }
    }

    model_poses.clear();
    for(it = pose_in_ports.begin(); it != pose_in_ports.end(); it++){
        model_poses.names.push_back(it->first);
        model_poses.elements.push_back(base::samples::RigidBodyState());
    }
}

const std::vector<base::samples::RigidBodyState>& RobotModelInterface::update(){

    base::samples::RigidBodyState model_pose;
    PoseInPortMap::iterator it;
    for(it = pose_in_ports.begin(); it != pose_in_ports.end(); it++){
        if(it->second->readNewest(model_pose) == RTT::NewData)
            model_poses[it->first] = model_pose;
    }
    return model_poses.elements;
}

void RobotModelInterface::addInputPort(const std::string port_name){

    if(!task_context->getPort(port_name)){ // Don't recreate ports
        PoseInPort* port = new PoseInPort(port_name);
        pose_in_ports[port_name] = port;
        task_context->ports()->addPort(port_name, *port);
    }
}

void RobotModelInterface::removeInputPort(const std::string port_name){
    if(pose_in_ports.count(port_name) > 0){
        task_context->ports()->removePort(port_name);
        delete pose_in_ports[port_name];
        pose_in_ports.erase(port_name);
    }
}

}
