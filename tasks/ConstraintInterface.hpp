#ifndef CONSTRAINTINTERFACE_HPP
#define CONSTRAINTINTERFACE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <base/samples/Joints.hpp>
#include <ctrl_types/RigidBodyStateSE3.hpp>

namespace wbc{

class Constraint;
class ConstraintStatus;
class RobotModel;

typedef std::shared_ptr<Constraint> ConstraintPtr;
typedef std::shared_ptr<RobotModel> RobotModelPtr;

typedef RTT::InputPort<base::samples::RigidBodyStateSE3> CartRefPort;
typedef RTT::InputPort<base::samples::Joints> JntRefPort;
typedef RTT::InputPort<base::VectorXd> WeightInPort;
typedef RTT::InputPort<double> ActivationPort;
typedef RTT::OutputPort<base::samples::RigidBodyStateSE3> CartStatusPort;
typedef RTT::OutputPort<base::samples::Joints> JntStatusPort;
typedef RTT::OutputPort<wbc::ConstraintStatus> ConstraintStatusPort;

typedef std::shared_ptr<CartRefPort> CartRefPortPtr;
typedef std::shared_ptr<JntRefPort> JntRefPortPtr;
typedef std::shared_ptr<WeightInPort> WeightInPortPtr;
typedef std::shared_ptr<ActivationPort> ActivationPortPtr;
typedef std::shared_ptr<CartStatusPort> CartStatusPortPtr;
typedef std::shared_ptr<JntStatusPort> JntStatusPortPtr;
typedef std::shared_ptr<ConstraintStatusPort> ConstraintStatusPortPtr;

/**
 * @brief The ConstraintInterface class contains I/O ports for each constraint
 */
class ConstraintInterface
{
public:
    ConstraintInterface(ConstraintPtr constraint,
                        RobotModelPtr _robot_model,
                        RTT::TaskContext* task_context);
    ~ConstraintInterface();

    ConstraintPtr constraint;
    RobotModelPtr robot_model;

    base::samples::RigidBodyStateSE3 constraint_cart_state;
    base::samples::Joints constraint_jnt_state;

    // Ports
    CartRefPortPtr cart_ref_port;
    JntRefPortPtr jnt_ref_port;
    CartStatusPortPtr cart_state_out_port;
    JntStatusPortPtr jnt_state_out_port;
    WeightInPortPtr weight_port;
    ActivationPortPtr activation_port;
    ConstraintStatusPortPtr constraint_status_port;

    base::samples::RigidBodyStateSE3 cart_ref; /** Cartesian Reference values */
    base::samples::Joints jnt_ref;          /** Jnt reference values */
    base::VectorXd weights;                 /** Current constraint weights*/
    double activation;                      /** Current constraint activation*/

    RTT::TaskContext* task_context;

    void update();
    void reset();
    void writeConstraintStatus(const ConstraintStatus& status);

};

}

#endif // CONSTRAINTINTERFACE_HPP
