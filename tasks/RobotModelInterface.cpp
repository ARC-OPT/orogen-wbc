#include "RobotModelInterface.hpp"
#include <wbc/robot_models/RobotModel.hpp>

namespace wbc{

RobotModelInterface::RobotModelInterface(RobotModel *model, RTT::TaskContext* task){
    robot_model = model;
    task_context = task;
}

RobotModelInterface::~RobotModelInterface(){
    for(size_t i = 0; i < pose_ports.size(); i++){
        task_context->ports()->removePort(pose_ports[i]->getName());
        delete pose_ports[i];
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

void RobotModelInterface::addPort(const std::string interface_name){
    RTT::InputPort<base::samples::RigidBodyState>* port = new RTT::InputPort<base::samples::RigidBodyState>(interface_name + "_pose");
    pose_ports.push_back(port);
    task_context->ports()->addPort(port->getName(), *port);
}

}
