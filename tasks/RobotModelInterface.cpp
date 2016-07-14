#include "RobotModelInterface.hpp"
#include <wbc/robot_models/RobotModel.hpp>

namespace wbc{

RobotModelInterface::RobotModelInterface(RTT::TaskContext* task){
    task_context = task;
}

RobotModelInterface::~RobotModelInterface(){
    for(size_t i = 0; i < pose_ports.size(); i++){
        task_context->ports()->removePort(pose_ports[i]->getName());
        delete pose_ports[i];
    }
}

void RobotModelInterface::configure(RobotModel *model,
                                    const std::vector<RobotModelConfig> &config){

    robot_model = model;

    for(size_t i = 0; i < config.size(); i++){
        // Don't create an input port if the hook is empty!
        if(!config[i].hook.empty())
            addPort(config[i].hook + "_pose");
    }

    std::map<std::string, int> tmp;
    for(size_t i = 0; i < config.size(); i++)
        tmp[config[i].hook + "_pose"] = i;

    for(size_t i = 0; i < pose_ports.size(); i++){

        const std::string &port_name = pose_ports[i]->getName();
        if(tmp.count(port_name) == 0){
            task_context->ports()->removePort(port_name);
            delete pose_ports[i];
            pose_ports.erase(i);
            i--;
        }
    }
}

void RobotModelInterface::update(const base::samples::Joints& joint_state){

    std::vector<base::samples::RigidBodyState> poses;
    base::samples::RigidBodyState pose;
    for(size_t i = 0; i < pose_ports.size(); i++){
        if(pose_ports[i]->readNewest(pose) == RTT::NewData)
            poses.push_back(pose);
    }

    robot_model->update(joint_state, poses);
}

void RobotModelInterface::addPort(const std::string port_name){

    // Don't recreate ports
    if(!task_context->getPort(port_name)){
        RTT::InputPort<base::samples::RigidBodyState>* port = new RTT::InputPort<base::samples::RigidBodyState>(port_name);
        pose_ports.push_back(port);
        task_context->ports()->addPort(port->getName(), *port);
    }
}

}
