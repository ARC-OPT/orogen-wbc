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

    base::VectorXd act_robot_velocity_, solver_output_eigen_;
    std::vector<TaskFrame> task_frames_;
    std::vector<SolverInput> solver_input_;
    base::samples::RigidBodyState constraint_pose_;
    base::Time stamp_;

    std::vector<std::string> joint_names_;
    base::samples::Joints joint_state_;
    base::commands::Joints solver_output_;
    bool debug_;

    void addPortsForConstraint(const ConstraintInterface* sti);
    void removePortsOfConstraint(const ConstraintInterface* sti);

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

