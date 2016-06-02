/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBCVELOCITYTASK_TASK_HPP
#define WBC_WBCVELOCITYTASK_TASK_HPP

#include "wbc/WbcVelocityTaskBase.hpp"
#include <wbc/models/KinematicRobotModelKDL.hpp>
#include <wbc/WbcVelocity.hpp>
#include <wbc/solvers/HierarchicalLeastSquaresSolver.hpp>

namespace wbc {

class WbcVelocityTask : public WbcVelocityTaskBase
{
    friend class WbcVelocityTaskBase;
protected:
    base::VectorXd joint_weights;
    base::VectorXd robot_vel; /** Robot velocity, converted from joint_state*/
    base::VectorXd damping, inv_condition_numbers, manipulability;
    std::vector<base::VectorXd> singular_values;
    bool compute_debug;
    std::vector<base::samples::RigidBodyState> task_frames;

public:
    WbcVelocityTask(std::string const& name = "wbc::WbcVelocity");
    WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~WbcVelocityTask(){}

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){WbcVelocityTaskBase::errorHook();}
    void stopHook();
    void cleanupHook();
};
}

#endif

