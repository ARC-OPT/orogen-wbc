/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBC_TASK_HPP
#define WBC_WBC_TASK_HPP

#include "wbc/WbcTaskBase.hpp"
#include <base/commands/Joints.hpp>

namespace wbc {

class WbcScene;
class RobotModel;
class ConstraintInterface;
class RobotModelInterface;

typedef std::shared_ptr<WbcScene> WbcScenePtr;
typedef std::shared_ptr<RobotModel> RobotModelPtr;
typedef std::shared_ptr<ConstraintInterface> ConstraintInterfacePtr;
typedef std::map<std::string, ConstraintInterfacePtr> ConstraintInterfaceMap;
typedef std::shared_ptr<RobotModelInterface> RobotModelInterfacePtr;

/**
 *
 * Base WBC Task.
 *
 *  configureHook:
 *
 *  1. Load and configure robot models
 *  2. Configure wbc scene
 *  3. Create dynamic ports
 *
 *  updateHook:
 *
 *  1. Read input ports
 *  2. Update robot models
 *  3. Update constraints and send them to the solver
 *
 */
class WbcTask : public WbcTaskBase
{
    friend class WbcBase;
protected:

    WbcScenePtr wbc_scene;
    RobotModelPtr robot_model;

    ConstraintInterfaceMap constraint_interfaces;      /** Contains I/O ports for each constraint*/
    RobotModelInterfacePtr robot_model_interface;      /** Contains I/O ports for the robot model(s)*/
    base::commands::Joints solver_output;              /** Solver output vector.*/
    base::samples::Joints joint_state;                 /** Current joint state of the whole robot*/
    base::Time stamp;                                  /** Timestamp for cycle time computation*/
    std::vector<ConstraintConfig> wbc_config;          /** Current constraint configuration*/

    /** Update constraints and send them to the solver*/
    virtual void updateConstraints() = 0;

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

    virtual void activateConstraint(const std::string& constraint_name, bool activate);
    virtual void activateConstraints(const std::vector<std::string>& constraint_names, bool activate);
    virtual void deactivateAllConstraints();
};
}

#endif

