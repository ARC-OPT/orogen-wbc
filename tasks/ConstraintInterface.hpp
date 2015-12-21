#ifndef CONSTRAINTINTERFACE_HPP
#define CONSTRAINTINTERFACE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace wbc{

class Constraint;

class ConstraintInterface
{
public:
    ConstraintInterface(Constraint* config, RTT::TaskContext* task);
    ~ConstraintInterface();

    void update(const base::samples::Joints& joint_state);
    void reset();

    Constraint *constraint;

    base::samples::RigidBodyState constraint_pose_;
    base::samples::Joints constraint_jnt_state_;

    RTT::InputPort<base::samples::RigidBodyState>* cart_ref_port;
    RTT::InputPort<base::samples::Joints>* jnt_ref_port;
    RTT::InputPort<base::VectorXd>* weight_port;
    RTT::InputPort<double>* activation_port;

    //Debug Ports
    RTT::OutputPort<base::samples::RigidBodyState>* pose_out_port;
    RTT::OutputPort<base::samples::Joints>* joint_state_out_port;

    base::samples::RigidBodyState cart_ref; /** Cartesian Reference values */
    base::samples::Joints jnt_ref;          /** Jnt reference values */

    RTT::TaskContext *task_context;
};

}

#endif // CONSTRAINTINTERFACE_HPP
