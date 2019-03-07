#ifndef KINEMATICMODELINTERFACE_HPP
#define KINEMATICMODELINTERFACE_HPP

#include <rtt/InputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <wbc_common/CartesianState.hpp>

namespace wbc{

class RobotModelConfig;

/**
 * @brief The RobotModelInterface class contains I/O ports for the robot model(s)
 */
class RobotModelInterface{
public:
    RobotModelInterface(RTT::TaskContext* task);
    ~RobotModelInterface();

    void configure(const std::vector<std::string> &names);
    std::vector<CartesianState> update();

protected:
    typedef RTT::InputPort<CartesianState> PoseInPort;
    typedef std::shared_ptr<PoseInPort> PoseInPortPtr;
    typedef std::map< std::string, PoseInPortPtr > PoseInPortMap;
    typedef RTT::OutputPort<CartesianState> PoseOutPort;
    typedef std::shared_ptr<PoseOutPort> PoseOutPortPtr;
    typedef std::map< std::string, PoseOutPortPtr > PoseOutPortMap;

    PoseInPortMap pose_in_ports;
    PoseOutPortMap pose_out_ports;
    CartesianState model_pose;
    RTT::TaskContext* task_context;

    void addInputPort(const std::string interface_name);
    void removeInputPort(const std::string port_name);
    void addOutputPort(const std::string port_name);
    void removeOutputPort(const std::string port_name);
};

}

#endif // KINEMATICMODELINTERFACE_HPP
