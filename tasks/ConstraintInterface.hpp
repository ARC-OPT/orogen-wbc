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
    ConstraintInterface(Constraint* config);
    ~ConstraintInterface();

    void update();
    void reset();
    void addPortsToTaskContext(RTT::TaskContext* task);
    void removePortsFromTaskContext(RTT::TaskContext* task);

    Constraint *constraint;

    base::samples::RigidBodyState constraint_pose_;

    RTT::InputPort<base::samples::RigidBodyState>* cart_ref_port;
    RTT::InputPort<base::samples::Joints>* jnt_ref_port;
    RTT::InputPort<base::VectorXd>* weight_port;
    RTT::InputPort<double>* activation_port;

    //Debug Ports
    RTT::OutputPort<base::samples::RigidBodyState>* pose_out_port;

    base::samples::RigidBodyState cart_ref; /** Cartesian Reference values */
    base::samples::Joints jnt_ref;          /** Jnt reference values */
};
typedef std::map< std::string, ConstraintInterface* > ConstraintInterfaceMap;

}

#endif // CONSTRAINTINTERFACE_HPP
