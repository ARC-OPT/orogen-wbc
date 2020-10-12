#include "RobotModelInterface.hpp"
#include <wbc/core/RobotModelConfig.hpp>
#include <urdf_parser/urdf_parser.h>

namespace wbc{

RobotModelInterface::RobotModelInterface(RTT::TaskContext* task){
    task_context = task;
}

RobotModelInterface::~RobotModelInterface(){
    pose_in_ports.clear();
}

void RobotModelInterface::configure(const std::vector<RobotModelConfig>& model_config){

    for(const RobotModelConfig& cfg : model_config){
        if(cfg.type == ModelType::VIRTUAL_6_DOF_JOINT){
            std::string name = getRobotName(cfg.file);
            addInputPort(name);
        }
    }

    // Remove ports which are not required anymore. This is required
    // if wbc is re-configured with different constraints
    for(auto it = pose_in_ports.begin(); it != pose_in_ports.end();){

        bool is_port_required = false;
        for(const RobotModelConfig& cfg : model_config){
            std::string name = getRobotName(cfg.file);
            if(it->first == (name))
                is_port_required = true;
        }
        if(is_port_required)
             it++;
        else{
            removeInputPort(it->first);
        }
    }
}

base::NamedVector<base::samples::RigidBodyStateSE3> RobotModelInterface::update(){

    base::samples::RigidBodyStateSE3 rbs;
    base::NamedVector<base::samples::RigidBodyStateSE3> state;
    for(const auto &it : pose_in_ports){
        if(it.second->readNewest(rbs) == RTT::NewData){
            state.elements.push_back(rbs);
            state.names.push_back(it.first);
        }
    }
    return state;
}

void RobotModelInterface::addInputPort(const std::string name){

    if(pose_in_ports.count(name) == 0){ // Don't recreate ports
        StateInPortPtr port = std::make_shared<StateInPort>();
        pose_in_ports[name] = port;
        task_context->ports()->addPort(name + "_state", *port);
    }
}

void RobotModelInterface::removeInputPort(const std::string name){
    if(pose_in_ports.count(name) > 0){
        task_context->ports()->removePort(name + "_state");
        pose_in_ports.erase(name);
    }
}

std::string RobotModelInterface::getRobotName(const std::string& urdf_file){
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(urdf_file);
    if (!urdf_model)
        throw std::invalid_argument("Cannot load URDF from file " + urdf_file);

    return urdf_model->getName();
}
}
