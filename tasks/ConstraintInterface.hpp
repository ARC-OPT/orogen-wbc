#ifndef CONSTRAINTINTERFACE_HPP
#define CONSTRAINTINTERFACE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace wbc{

class Constraint;
class RobotModel;

typedef std::shared_ptr<Constraint> ConstraintPtr;
typedef std::shared_ptr<RobotModel> RobotModelPtr;

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

    base::samples::RigidBodyState constraint_cart_state;
    base::samples::Joints constraint_jnt_state;

    // Ports
    RTT::InputPort<base::samples::RigidBodyState>* cart_ref_port;
    RTT::InputPort<base::samples::Joints>* jnt_ref_port;
    RTT::OutputPort<base::samples::RigidBodyState>* cart_state_out_port;
    RTT::OutputPort<base::samples::Joints>* jnt_state_out_port;
    RTT::OutputPort<Constraint> *constraint_out_port;
    RTT::InputPort<base::VectorXd>* weight_port;
    RTT::InputPort<double>* activation_port;

    base::samples::RigidBodyState cart_ref; /** Cartesian Reference values */
    base::samples::Joints jnt_ref;          /** Jnt reference values */
    base::VectorXd weights;                 /** Current constraint weights*/
    double activation;                      /** Current constraint activation*/

    RTT::TaskContext* task_context;

    void update();
    void reset();
    void writeDebug();

};

}

#endif // CONSTRAINTINTERFACE_HPP
