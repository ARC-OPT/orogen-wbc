#include "RobotModelInterface.hpp"
#include <wbc/RobotModelConfig.hpp>

namespace wbc{

RobotModelInterface::RobotModelInterface(RTT::TaskContext* task){
    task_context = task;
}

RobotModelInterface::~RobotModelInterface(){
    pose_in_ports.clear();
}

void RobotModelInterface::configure(const std::vector<std::string> &names){

    for(const std::string& n : names)
        addInputPort(n);

    // Remove ports which are not required anymore. This is required
    // if wbc is re-configured with different constraints
    for(auto it = pose_in_ports.begin(); it != pose_in_ports.end();){

        bool is_port_required = false;
        for(const std::string &n : names){
            if(it->first == (n))
                is_port_required = true;
        }
        if(is_port_required)
             it++;
        else
            removeInputPort(it->first);
    }

    models_state.clear();
    for(auto it = pose_in_ports.begin(); it != pose_in_ports.end(); it++){
        models_state.names.push_back(it->first);
        models_state.elements.push_back(wbc::CartesianState());
    }
}

std::vector<CartesianState> RobotModelInterface::update(){

    wbc::CartesianState model_pose;
    for(const auto &it : pose_in_ports){
        if(it.second->readNewest(model_pose) == RTT::NewData){
            model_pose.source_frame = it.first;
            models_state[it.first] = model_pose;
        }
    }
    return models_state.elements;
}

void RobotModelInterface::addInputPort(const std::string name){

    if(pose_in_ports.count(name) == 0){ // Don't recreate ports
        PoseInPortPtr port = std::make_shared<PoseInPort>();
        pose_in_ports[name] = port;
        task_context->ports()->addPort(name + "_pose", *port);
    }
}

void RobotModelInterface::removeInputPort(const std::string name){
    if(pose_in_ports.count(name) > 0){
        task_context->ports()->removePort(name + "_pose");
        pose_in_ports.erase(name);
    }
}
}
