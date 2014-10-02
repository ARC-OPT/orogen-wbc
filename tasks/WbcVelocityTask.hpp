/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBCVELOCITYTASK_TASK_HPP
#define WBC_WBCVELOCITYTASK_TASK_HPP

#include "wbc/WbcVelocityTaskBase.hpp"
#include "ConstraintInterface.hpp"


namespace wbc {

class WbcVelocity;

class WbcVelocityTask : public WbcVelocityTaskBase
{
    friend class WbcVelocityTaskBase;
protected:
    WbcVelocity *wbc_; /** This will create the equation system for the solver*/

    ConstraintInterfaceMap constraint_interface_map_; /** Containts port interfaces for each constraint*/
    std::vector<ConstraintsPerPrio> constraints_;   /** Contains all constraints, sorted by priority */
    std::vector<TaskFrame> task_frames_; /** Task frames coming from the robot model*/
    std::vector<LinearEqnSystem> equations_; /** Equation system created here, input for the solver*/
    base::VectorXd joint_weights_;
    base::commands::Joints ctrl_out_; /** Control output */
    base::samples::Joints joint_state_; /** Optional joint status input. Is used to compute the constraint status*/
    base::VectorXd solver_output_; /** Solution coming from the solver*/
    base::VectorXd robot_vel_; /** Robot velocity, converted from joint_state*/

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

