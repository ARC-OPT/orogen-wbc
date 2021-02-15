/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBC_TASK_HPP
#define WBC_WBC_TASK_HPP

#include "wbc/WbcTaskBase.hpp"
#include <base/commands/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <wbc/core/ConstraintStatus.hpp>
#include <wbc/tools/JointIntegrator.hpp>
#include <wbcTypes.hpp>

namespace wbc {

class WbcScene;
class RobotModel;
class QPSolver;
class ConstraintInterface;
class HierarchicalQP;

typedef std::shared_ptr<WbcScene> WbcScenePtr;
typedef std::shared_ptr<RobotModel> RobotModelPtr;
typedef std::shared_ptr<QPSolver> QPSolverPtr;
typedef std::shared_ptr<ConstraintInterface> ConstraintInterfacePtr;
typedef std::map<std::string, ConstraintInterfacePtr> ConstraintInterfaceMap;

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
    QPSolverPtr solver;
    JointIntegrator integrator;

    HierarchicalQP hierarchical_qp;
    ConstraintInterfaceMap constraint_interfaces;         /** Contains I/O ports for each constraint*/
    base::commands::Joints solver_output_joints;          /** Solver output vector.*/
    base::samples::Joints joint_state;                    /** Current joint state of the whole robot (only actuated joints)*/
    base::samples::Joints full_joint_state;               /** Current joint state of the whole robot (all joints)*/
    base::Time stamp;                                     /** Timestamp for cycle time computation*/
    ConstraintsStatus constraints_status;                 /** Status of constraints*/
    base::samples::RigidBodyStateSE3 floating_base_state; /** Current status of the floating base*/
    base::samples::RigidBodyState floating_base_state_rbs;/** Deprecated floating base state*/
    wbc::JointWeights joint_weights;                     /** Current joint weights*/
    std::vector<ConstraintConfig> wbc_config;             /** WBC constraint configuration*/
    bool compute_constraint_status;                       /** For debugging purpose*/
    bool integrate;                                       /** Perform numerical integration for the solver output*/
    TimingStats timing_stats;                             /** statistics on compuation time*/

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

    virtual void activateConstraint(const std::string& constraint_name, double activation);
    virtual void activateConstraints(const std::vector<std::string>& constraint_names, double activation);
    virtual void deactivateAllConstraints();
};
}

#endif

