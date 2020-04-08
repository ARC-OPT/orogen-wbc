#ifndef KINEMATICMODELINTERFACE_HPP
#define KINEMATICMODELINTERFACE_HPP

#include <rtt/InputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>
#include <wbc/core/RobotModelConfig.hpp>

namespace wbc{

/**
 * @brief The RobotModelInterface class contains I/O ports for the robot model(s)
 */
class RobotModelInterface{
public:
    RobotModelInterface(RTT::TaskContext* task);
    ~RobotModelInterface();

    void configure(const RobotModelsState &initial_states);
    RobotModelsState update();
    RobotModelsState getModelsState(){return models_state;}

protected:
    typedef RTT::InputPort<base::samples::RigidBodyStateSE3> PoseInPort;
    typedef std::shared_ptr<PoseInPort> PoseInPortPtr;
    typedef std::map< std::string, PoseInPortPtr > PoseInPortMap;
    typedef RTT::OutputPort<base::samples::RigidBodyStateSE3> PoseOutPort;
    typedef std::shared_ptr<PoseOutPort> PoseOutPortPtr;
    typedef std::map< std::string, PoseOutPortPtr > PoseOutPortMap;

    PoseInPortMap pose_in_ports;
    RobotModelsState models_state;
    RTT::TaskContext* task_context;

    void addInputPort(const std::string interface_name);
    void removeInputPort(const std::string port_name);
};

}

#endif // KINEMATICMODELINTERFACE_HPP
