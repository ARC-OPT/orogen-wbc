#include "KinematicModelInterface.hpp"
#include <wbc/KinematicModel.hpp>

namespace wbc{

KinematicModelInterface::KinematicModelInterface(KinematicModel *model, const std::string interface_name, RTT::TaskContext* task){
    kinematic_model = model;
    task_context = task;

    model_pose_port = new RTT::InputPort<base::samples::RigidBodyState>(interface_name + "_pose");
    task_context->ports()->addPort(model_pose_port->getName(), *model_pose_port);
}

KinematicModelInterface::~KinematicModelInterface(){
    task_context->ports()->removePort(model_pose_port->getName());
}

void KinematicModelInterface::update(){
    if(model_pose_port->read(model_pose) == RTT::NewData)
        kinematic_model->updateLink(model_pose);
}

}
