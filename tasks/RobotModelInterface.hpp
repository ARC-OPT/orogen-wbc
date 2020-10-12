#ifndef KINEMATICMODELINTERFACE_HPP
#define KINEMATICMODELINTERFACE_HPP

#include <rtt/InputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <wbc/core/RobotModelConfig.hpp>
#include <base/NamedVector.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

namespace wbc{

/**
 * @brief The RobotModelInterface class contains I/O ports for the robot model(s)
 */
class RobotModelInterface{
public:
    RobotModelInterface(RTT::TaskContext* task);
    ~RobotModelInterface();

    void configure(const std::vector<RobotModelConfig>& model_config);
    base::NamedVector<base::samples::RigidBodyStateSE3> update();

protected:
    typedef RTT::InputPort<base::samples::RigidBodyStateSE3> StateInPort;
    typedef std::shared_ptr<StateInPort> StateInPortPtr;
    typedef std::map< std::string, StateInPortPtr > StateInPortMap;

    StateInPortMap pose_in_ports;
    RTT::TaskContext* task_context;

    void addInputPort(const std::string interface_name);
    void removeInputPort(const std::string port_name);
    std::string getRobotName(const std::string& urdf_file);
};

}

#endif // KINEMATICMODELINTERFACE_HPP
