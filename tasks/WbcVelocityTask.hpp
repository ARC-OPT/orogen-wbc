/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBCVELOCITYTASK_TASK_HPP
#define WBC_WBCVELOCITYTASK_TASK_HPP

#include "wbc/WbcVelocityTaskBase.hpp"
#include <wbc/wbcTypes.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <wbc/WbcVelocity.hpp>
#include <base/logging.h>
#include <wbc/Constraint.hpp>
#include <wbc/PriorityData.hpp>

namespace wbc {

class ConstraintInterface
{
public:
    ConstraintInterface(Constraint* config);
    ~ConstraintInterface();

    void update();
    void reset();

    Constraint *constraint;

    RTT::InputPort<base::samples::RigidBodyState>* cart_ref_port;
    RTT::InputPort<base::samples::Joints>* jnt_ref_port;
    RTT::InputPort<base::VectorXd>* weight_port;
    RTT::InputPort<double>* activation_port;

    //Debug Ports
    RTT::OutputPort<base::samples::RigidBodyState>* pose_out_port;
    RTT::OutputPort<Constraint>* constraint_out_port;

    base::samples::RigidBodyState cart_ref; /** Cartesian Reference values */
    base::samples::Joints jnt_ref;          /** Jnt reference values */
};
typedef std::map< std::string, ConstraintInterface* > ConstraintInterfaceMap;


class WbcVelocityTask : public WbcVelocityTaskBase
{
    friend class WbcVelocityTaskBase;
protected:
    WbcVelocity wbc_;

    ConstraintInterfaceMap constraint_interface_map_;

    base::VectorXd joint_weights_;
    base::VectorXd solver_output_, act_robot_velocity_;
    base::samples::Joints ctrl_out_;
    base::samples::Joints joint_status_;
    bool debug_;
    std::vector<PriorityData> prio_data_;
    std::vector<RTT::OutputPort<PriorityData>*> prio_data_ports_;

    void addPortsForConstraint(const ConstraintInterface* sti)
    {
        if(sti->constraint->config.type == wbc::cart){
            ports()->addPort(sti->cart_ref_port->getName(), *(sti->cart_ref_port));
            ports()->addPort(sti->pose_out_port->getName(), *(sti->pose_out_port));
        }
        else
            ports()->addPort((sti->jnt_ref_port)->getName(), *(sti->jnt_ref_port));

        ports()->addPort(sti->weight_port->getName(), *(sti->weight_port));
        ports()->addPort(sti->activation_port->getName(), *(sti->activation_port));

        if(debug_)
            ports()->addPort(sti->constraint_out_port->getName(), *(sti->constraint_out_port));
    }

    void removePortsOfConstraint(const ConstraintInterface* sti)
    {
        ports()->removePort(sti->weight_port->getName());
        ports()->removePort(sti->activation_port->getName());

        if(debug_)
        {
            if(ports()->getPort(sti->constraint_out_port->getName())){
                ports()->removePort(sti->constraint_out_port->getName());
            }
        }
        if(sti->constraint->config.type == cart)
        {
            if(ports()->getPort(sti->pose_out_port->getName())){
                ports()->removePort(sti->pose_out_port->getName());
            }
            if(ports()->getPort(sti->cart_ref_port->getName())){
                ports()->removePort(sti->cart_ref_port->getName());
            }
        }
        else{
            if(ports()->getPort(sti->jnt_ref_port->getName())){
                ports()->removePort(sti->jnt_ref_port->getName());
            }
        }
    }

public:
    WbcVelocityTask(std::string const& name = "wbc::WbcVelocity");
    WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~WbcVelocityTask(){}

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){WbcVelocityTaskBase::errorHook();}
    void stopHook(){WbcVelocityTaskBase::stopHook();}
    void cleanupHook();
};
}

#endif

