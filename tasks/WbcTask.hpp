/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBC_TASK_HPP
#define WBC_WBC_TASK_HPP

#include "wbc/WbcTaskBase.hpp"
#include "ConstraintInterface.hpp"
#include "RobotModelInterface.hpp"
#include <wbc/OptProblem.hpp>
#include <wbc/TaskFrame.hpp>

namespace wbc {

class Wbc;
class RobotModel;
class Solver;

typedef std::map<std::string, ConstraintInterface*> ConstraintInterfaceMap;

class WbcTask : public WbcTaskBase
{
    friend class WbcBase;
protected:

    Wbc* wbc;
    RobotModel* robot_model;
    Solver* solver;

    ConstraintInterfaceMap constraint_interfaces; /** Contains I/O ports for each constraint*/
    RobotModelInterface* robot_model_interface;   /** Contains I/O ports for the robot model*/
    base::commands::Joints ctrl_out;              /** Control output vector.*/
    base::samples::Joints joint_state;            /** Current joint state of the whole robot*/
    base::Time stamp;                             /** Timestamp for cycle time computation*/
    HierarchicalWeightedLS opt_problem;           /** Current optimization problem*/
    Eigen::VectorXd solver_output;                /** Output of the solver. Size is same as control output*/
    std::vector<TaskFrame> task_frames_out;       /** Debug output: The current task frames*/

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

