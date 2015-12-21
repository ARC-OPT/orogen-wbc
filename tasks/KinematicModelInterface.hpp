#ifndef KINEMATICMODELINTERFACE_HPP
#define KINEMATICMODELINTERFACE_HPP

#include <rtt/InputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace wbc{

class KinematicModel;

class KinematicModelInterface{
public:
    KinematicModelInterface(KinematicModel *model, const std::string interface_name, RTT::TaskContext* task);
    ~KinematicModelInterface();

    void update();

protected:
    RTT::InputPort<base::samples::RigidBodyState>* model_pose_port;
    base::samples::RigidBodyState model_pose;
    KinematicModel* kinematic_model;
    RTT::TaskContext* task_context;
};

}

#endif // KINEMATICMODELINTERFACE_HPP
