/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBCVELOCITYTASK_TASK_HPP
#define WBC_WBCVELOCITYTASK_TASK_HPP

#include "wbc/WbcVelocityTaskBase.hpp"
#include "ConstraintInterface.hpp"
#include <wbc/TaskFrameKDL.hpp>
#include <base/commands/Joints.hpp>

namespace wbc {

class HierarchicalWDLSSolver;
class RobotModelKDL;
class WbcVelocity;

class WbcVelocityTask : public WbcVelocityTaskBase
{
    friend class WbcVelocityTaskBase;
protected:

    ConstraintInterfaceMap constraint_interface_map_; /** Containts port interfaces for each constraint*/
    std::vector<ConstraintsPerPrio> constraints_;   /** Contains all constraints, sorted by priority */
    std::vector<TaskFrameKDL> task_frames_; /** Task frames coming from the robot model*/
    std::vector<LinearEqnSystem> equations_; /** Equation system, input for the solver*/
    base::VectorXd joint_weights_;
    base::commands::Joints ctrl_out_; /** Control output */
    base::samples::Joints joint_state_; /** Optional joint status input. Is used to compute the constraint status*/
    base::VectorXd solver_output_; /** Solution coming from the solver*/
    base::VectorXd robot_vel_; /** Robot velocity, converted from joint_state*/
    base::VectorXd damping_, inv_condition_numbers_, manipulability_;
    std::vector<base::VectorXd> singular_values_;
    base::Time stamp_;
    bool compute_debug_;

    WbcVelocity *wbc_; /** This will create the equation system for the solver*/
    HierarchicalWDLSSolver* solver_;
    RobotModelKDL *robot_model_;

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

