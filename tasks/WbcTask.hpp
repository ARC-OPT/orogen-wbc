/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBC_TASK_HPP
#define WBC_WBC_TASK_HPP

#include "wbc/WbcTaskBase.hpp"
#include <wbc/constraints/ConstraintConfig.hpp>
#include "ConstraintInterface.hpp"
#include "RobotModelInterface.hpp"

namespace wbc {

class Wbc;
class RobotModel;
class Solver;
class OptProblem;

class WbcTask : public WbcTaskBase
{
    friend class WbcBase;
protected:

    Wbc* wbc;
    RobotModel* robot_model;
    Solver* solver;

    std::vector<ConstraintInterface*> constraint_interfaces; /** Contains I/O ports for each constraint*/
    RobotModelInterface* robot_model_interface;
    base::commands::Joints ctrl_out;
    base::samples::Joints joint_state;
    base::Time stamp;
    std::vector<OptProblem*> opt_problem;
    Eigen::VectorXd solver_output;

public:
    WbcTask(std::string const& name = "wbc::WbcTask");
    WbcTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~WbcTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

