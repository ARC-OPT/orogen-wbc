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
    PoseOutPortMap::iterator oit;
    for(oit = pose_out_ports.begin(); oit != pose_out_ports.end(); oit++){
        task_context->ports()->removePort(oit->second->getName());
        delete oit->second;
    }
}

void RobotModelInterface::configure(const std::vector<RobotModelConfig> &config){

    for(size_t i = 0; i < config.size(); i++)
        if(!config[i].hook.empty()){  // Don't create an input port if the hook is empty
            addInputPort(config[i].initial_pose.sourceFrame);
            addOutputPort(config[i].initial_pose.sourceFrame);
        }

    // Remove ports which are not required anymore. This is required
    // if wbc is re-configured with different constraints
    PoseInPortMap::iterator it;
    for(it = pose_in_ports.begin(); it != pose_in_ports.end(); it++){

        bool is_port_required = false;
        for(size_t j = 0; j < config.size(); j++){
            if(it->first == (config[j].initial_pose.sourceFrame))
                is_port_required = true;
        }
        if(!is_port_required){
            removeInputPort(it->first);
            it--;
        }
    }
    PoseOutPortMap::iterator oit;
    for(oit = pose_out_ports.begin(); oit != pose_out_ports.end(); oit++){

        bool is_port_required = false;
        for(size_t j = 0; j < config.size(); j++){
            if(oit->first == (config[j].initial_pose.sourceFrame))
                is_port_required = true;
        }
        if(!is_port_required){
            removeOutputPort(oit->first);
            oit--;
        }
    }
}

std::vector<base::samples::RigidBodyState> RobotModelInterface::update(){

    std::vector<base::samples::RigidBodyState> poses;
    PoseInPortMap::iterator it;
    for(it = pose_in_ports.begin(); it != pose_in_ports.end(); it++){
        if(it->second->readNewest(model_pose) == RTT::NewData){
            model_pose.sourceFrame = it->first;
            poses.push_back(model_pose);
            pose_out_ports[it->first]->write(model_pose);
        }
    }
    return poses;
}

void RobotModelInterface::addInputPort(const std::string name){

    if(pose_in_ports.count(name) == 0){ // Don't recreate ports
        PoseInPort* port = new PoseInPort();
        pose_in_ports[name] = port;
        task_context->ports()->addPort(name + "_pose", *port);
    }
}

void RobotModelInterface::removeInputPort(const std::string name){
    if(pose_in_ports.count(name) > 0){
        task_context->ports()->removePort(name + "_pose");
        delete pose_in_ports[name];
        pose_in_ports.erase(name);
    }
}

void RobotModelInterface::addOutputPort(const std::string name){

    if(pose_out_ports.count(name) == 0){ // Don't recreate ports
        PoseOutPort* port = new PoseOutPort();
        pose_out_ports[name] = port;
        task_context->ports()->addPort("current_" + name + "_pose", *port);
    }
}

void RobotModelInterface::removeOutputPort(const std::string name){
    if(pose_out_ports.count(name) > 0){
        task_context->ports()->removePort("current_" + name + "_pose");
        delete pose_out_ports[name];
        pose_out_ports.erase(name);
    }
}

}
