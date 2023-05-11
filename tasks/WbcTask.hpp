/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBC_TASK_HPP
#define WBC_WBC_TASK_HPP

#include "wbc/WbcTaskBase.hpp"
#include <base/commands/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Wrenches.hpp>
#include <wbc/core/TaskStatus.hpp>
#include <wbc/tools/JointIntegrator.hpp>
#include <wbcTypes.hpp>

namespace wbc {

class Scene;
class RobotModel;
class QPSolver;
class TaskInterface;
class HierarchicalQP;

typedef std::shared_ptr<Scene> ScenePtr;
typedef std::shared_ptr<RobotModel> RobotModelPtr;
typedef std::shared_ptr<QPSolver> QPSolverPtr;
typedef std::shared_ptr<TaskInterface> TaskInterfacePtr;
typedef std::map<std::string, TaskInterfacePtr> TaskInterfaceMap;

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
 *  3. Update Tasks and send them to the solver
 *
 */
class WbcTask : public WbcTaskBase
{
    friend class WbcBase;
protected:

    ScenePtr wbc_scene;
    RobotModelPtr robot_model;
    QPSolverPtr solver;
    JointIntegrator integrator;

    HierarchicalQP hierarchical_qp;
    TaskInterfaceMap task_interfaces;                     /** Contains I/O ports for each task*/
    base::commands::Joints solver_output_joints;          /** Solver output vector.*/
    base::samples::Joints joint_state;                    /** Current joint state of the whole robot (only actuated joints)*/
    base::samples::Joints full_joint_state;               /** Current joint state of the whole robot (all joints)*/
    base::Time stamp;                                     /** Timestamp for cycle time computation*/
    TasksStatus tasks_status;                             /** Status of all tasks*/
    base::samples::RigidBodyStateSE3 floating_base_state; /** Current status of the floating base*/
    base::samples::RigidBodyState floating_base_state_rbs;/** Deprecated floating base state*/
    wbc::JointWeights joint_weights;                      /** Current joint weights*/
    std::vector<TaskConfig> wbc_config;                   /** WBC tasks configuration*/
    bool compute_task_status;                             /** For debugging purpose*/
    bool integrate;                                       /** Perform numerical integration for the solver output*/
    TimingStats timing_stats;                             /** statistics on compuation time*/
    ActiveContacts active_contacts;                       /** Names of the active contact points*/
    base::samples::Wrenches contact_wrenches;             /** Measured contact wrenches*/
    bool has_floating_base_state;                         /** Is floating base state available on inpurt port?*/

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

    virtual void activateTask(const std::string& task_name, double activation);
    virtual void activateTasks(const std::vector<std::string>& task_names, double activation);
    virtual void deactivateAllTasks();
};
}

#endif

