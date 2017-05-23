/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBC_TASK_HPP
#define WBC_WBC_TASK_HPP

#include "wbc/WbcTaskBase.hpp"
#include <base/commands/Joints.hpp>

namespace wbc {

class WbcScene;
class RobotModel;
class Solver;
class ConstraintInterface;
class RobotModelInterface;

typedef std::map<std::string, ConstraintInterface*> ConstraintInterfaceMap;

class WbcTask : public WbcTaskBase
{
    friend class WbcBase;
protected:

    WbcScene* wbc_scene;
    RobotModel* robot_model;
    Solver* solver;

    ConstraintInterfaceMap constraint_interfaces; /** Contains I/O ports for each constraint*/
    RobotModelInterface* robot_model_interface;   /** Contains I/O ports for the robot model*/
    base::commands::Joints ctrl_out;              /** Control output vector.*/
    base::samples::Joints joint_state;            /** Current joint state of the whole robot*/
    base::Time stamp;                             /** Timestamp for cycle time computation*/
    base::VectorXd joint_weights;                 /** Joint weights of the whole robot*/

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

