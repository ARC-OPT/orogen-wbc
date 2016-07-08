#ifndef CONSTRAINTINTERFACE_HPP
#define CONSTRAINTINTERFACE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace wbc{

class Wbc;
class RobotModel;
class Constraint;

class ConstraintInterface
{
public:
    ConstraintInterface(const std::string &constraint_name,
                        Wbc* wbc,
                        RobotModel* robot_model,
                        RTT::TaskContext* task_context);
    ~ConstraintInterface();

    Wbc* wbc;
    RobotModel* robot_model;

    base::samples::RigidBodyState constraint_cart_state;
    base::samples::Joints constraint_jnt_state;
    std::string constraint_name;

    // Ports
    RTT::InputPort<base::samples::RigidBodyState>* cart_ref_port;
    RTT::InputPort<base::samples::Joints>* jnt_ref_port;
    RTT::OutputPort<base::samples::RigidBodyState>* cart_state_out_port;
    RTT::OutputPort<base::samples::Joints>* jnt_state_out_port;
    RTT::OutputPort<wbc::Constraint>* constraint_out_port;
    RTT::InputPort<base::VectorXd>* weight_port;
    RTT::InputPort<double>* activation_port;

    base::samples::RigidBodyState cart_ref; /** Cartesian Reference values */
    base::samples::Joints jnt_ref;          /** Jnt reference values */
    base::VectorXd weights;                 /** Current constraint weights*/
    double activation;                      /** Current constraint activation*/

    RTT::TaskContext *task_context;

    void update();
    void reset();

};

}

#endif // CONSTRAINTINTERFACE_HPP
