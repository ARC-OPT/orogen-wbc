#ifndef KINEMATICMODELINTERFACE_HPP
#define KINEMATICMODELINTERFACE_HPP

#include <rtt/InputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Joints.hpp>

namespace wbc{

class RobotModel;

class RobotModelInterface{
public:
    RobotModelInterface(RTT::TaskContext* task);
    ~RobotModelInterface();

    void setRobotModel(RobotModel *model);
    void update(const base::samples::Joints& joint_state);
    void addPort(const std::string interface_name);

protected:
    std::vector< RTT::InputPort<base::samples::RigidBodyState>* > pose_ports;
    base::samples::RigidBodyState model_pose;
    RobotModel* robot_model;
    RTT::TaskContext* task_context;
};

}

#endif // KINEMATICMODELINTERFACE_HPP
