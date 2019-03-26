#ifndef KINEMATICMODELINTERFACE_HPP
#define KINEMATICMODELINTERFACE_HPP

#include <rtt/InputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <wbc/RobotModelConfig.hpp>

namespace wbc{

/**
 * @brief The RobotModelInterface class contains I/O ports for the robot model(s)
 */
class RobotModelInterface{
public:
    RobotModelInterface(RTT::TaskContext* task);
    ~RobotModelInterface();

    void configure(const std::vector<RobotModelConfig> &config);
    const std::vector<base::samples::RigidBodyState> &update();
    RobotModelPoses getModelPoses(){return model_poses;}

protected:
    typedef RTT::InputPort<base::samples::RigidBodyState> PoseInPort;
    typedef std::map< std::string, PoseInPort* > PoseInPortMap;

    PoseInPortMap pose_in_ports;
    RobotModelPoses model_poses;
    RTT::TaskContext* task_context;

    void addInputPort(const std::string interface_name);
    void removeInputPort(const std::string port_name);
};

}

#endif // KINEMATICMODELINTERFACE_HPP
