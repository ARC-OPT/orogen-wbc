#ifndef KINEMATICMODELINTERFACE_HPP
#define KINEMATICMODELINTERFACE_HPP

#include <rtt/InputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace wbc{

class RobotModelConfig;

class RobotModelInterface{
public:
    RobotModelInterface(RTT::TaskContext* task);
    ~RobotModelInterface();

    void configure(const std::vector<RobotModelConfig> &config);
    std::vector<base::samples::RigidBodyState> update();

protected:
    typedef RTT::InputPort<base::samples::RigidBodyState> PosePort;
    typedef std::map< std::string, PosePort* > PosePortMap;

    PosePortMap pose_ports;
    PosePortMap::iterator it;
    base::samples::RigidBodyState model_pose;
    RTT::TaskContext* task_context;

    void addPort(const std::string interface_name);
    void removePort(const std::string port_name);
};

}

#endif // KINEMATICMODELINTERFACE_HPP
