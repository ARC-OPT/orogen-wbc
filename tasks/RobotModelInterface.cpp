#include "RobotModelInterface.hpp"
#include <wbc/core/RobotModelConfig.hpp>

namespace wbc{

RobotModelInterface::RobotModelInterface(RTT::TaskContext* task){
    task_context = task;
}

RobotModelInterface::~RobotModelInterface(){
    pose_in_ports.clear();
}

void RobotModelInterface::configure(const base::samples::RigidBodyStatesSE3 &initial_states){

    for(const std::string& n : initial_states.names)
        addInputPort(n);

    // Remove ports which are not required anymore. This is required
    // if wbc is re-configured with different constraints
    for(auto it = pose_in_ports.begin(); it != pose_in_ports.end();){

        bool is_port_required = false;
        for(const std::string &n : initial_states.names){
            if(it->first == (n))
                is_port_required = true;
        }
        if(is_port_required)
             it++;
        else
            removeInputPort(it->first);
    }

    models_state = initial_states;
}

base::samples::RigidBodyStatesSE3 RobotModelInterface::update(){

    base::samples::RigidBodyStateSE3 model_pose;
    for(const auto &it : pose_in_ports){
        if(it.second->readNewest(model_pose) == RTT::NewData){
            if(models_state.time.isNull())
                models_state.time = model_pose.time;
            else if(model_pose.time > models_state.time)
                models_state.time = model_pose.time;
            models_state[it.first] = model_pose;
        }
    }
    return models_state;
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
