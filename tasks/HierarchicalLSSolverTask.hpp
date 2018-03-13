/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_HIERARCHICALLSSOLVERTASK_TASK_HPP
#define WBC_HIERARCHICALLSSOLVERTASK_TASK_HPP

#include "wbc/HierarchicalLSSolverTaskBase.hpp"
#include <wbcTypes.hpp>

namespace wbc{

class HierarchicalLSSolver;

/**
 *
 * @brief Hierarchical Least squares solver. Solves the problem of finding the minimum norm joint velocities that satify the constraints
 *
 *     A_w,1 * dq = y_1
 *     A_w,2 * dq = y_2
 *          ...
 *     A_w,P * dq = y_P
 *
 * where A_p are the weighted constraint Jacobians, dq the reference joint velocity and y_p the input for the robot task with priority p. Usually,
 * y_p are control outputs from feedback controllers that try to fulfil a certain task, e.g. maintaining a robot position in joint or
 * Cartesian space.
 */
class HierarchicalLSSolverTask : public HierarchicalLSSolverTaskBase
{
friend class HierarchicalLSSolverTaskBase;
protected:

    std::shared_ptr<HierarchicalLSSolver> solver; /** Pointer to the solver*/
    HierarchicalLEConstraints constraints_prio;   /** Input constraints, sorted by priority*/
    base::VectorXd solver_output_raw;             /** Raw output of the solver: Joint velocities*/
    base::VectorXd joint_weights;                 /** Joint weights of the whole robot*/
    base::VectorXd max_solver_output;             /** Maximum joint velocity (in rad/sec) for each joint*/

    /** Compute and write the control solution*/
    virtual void computeSolverOutput(base::commands::Joints& solver_output);

public:
    HierarchicalLSSolverTask(std::string const& name = "wbc::HierarchicalLSSolverTask");
    HierarchicalLSSolverTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~HierarchicalLSSolverTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};

}

#endif

