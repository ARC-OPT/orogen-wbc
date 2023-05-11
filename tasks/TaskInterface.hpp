#ifndef TASKINTERFACE_HPP
#define TASKINTERFACE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>
#include <wbc/core/TaskConfig.hpp>

namespace wbc{

class TaskStatus;
class RobotModel;
class Scene;

typedef std::shared_ptr<RobotModel> RobotModelPtr;
typedef std::shared_ptr<Scene> ScenePtr;

typedef RTT::InputPort<base::samples::RigidBodyStateSE3> CartRefPort;
typedef RTT::InputPort<base::samples::Joints> JntRefPort;
typedef RTT::InputPort<base::VectorXd> WeightInPort;
typedef RTT::InputPort<double> ActivationPort;
typedef RTT::OutputPort<base::samples::RigidBodyStateSE3> CartStatusPort;
typedef RTT::OutputPort<base::samples::Joints> JntStatusPort;
typedef RTT::OutputPort<wbc::TaskStatus> TaskStatusPort;

typedef std::shared_ptr<CartRefPort> CartRefPortPtr;
typedef std::shared_ptr<JntRefPort> JntRefPortPtr;
typedef std::shared_ptr<WeightInPort> WeightInPortPtr;
typedef std::shared_ptr<ActivationPort> ActivationPortPtr;
typedef std::shared_ptr<CartStatusPort> CartStatusPortPtr;
typedef std::shared_ptr<JntStatusPort> JntStatusPortPtr;
typedef std::shared_ptr<TaskStatusPort> TaskStatusPortPtr;

/**
 * @brief The TaskInterface class contains I/O ports for each task
 */
class TaskInterface
{
public:
    TaskInterface(TaskConfig _cfg,
                        ScenePtr _scene,
                        RobotModelPtr _robot_model,
                        RTT::TaskContext* _task_context);
    ~TaskInterface();

    TaskConfig cfg;
    RobotModelPtr robot_model;
    ScenePtr scene;

    base::samples::RigidBodyStateSE3 task_cart_state;
    base::samples::Joints task_jnt_state;

    // Ports
    CartRefPortPtr cart_ref_port;
    JntRefPortPtr jnt_ref_port;
    CartStatusPortPtr cart_state_out_port;
    JntStatusPortPtr jnt_state_out_port;
    WeightInPortPtr weight_port;
    ActivationPortPtr activation_port;
    TaskStatusPortPtr task_status_port;

    base::samples::RigidBodyStateSE3 cart_ref; /** Cartesian Reference values */
    base::samples::Joints jnt_ref;          /** Jnt reference values */
    base::VectorXd weights;                 /** Current task weights*/
    double activation;                      /** Current task activation*/

    RTT::TaskContext* task_context;

    void update();
    void writeTaskStatus(const TaskStatus& status);

};

}

#endif // TASKINTERFACE_HPP
