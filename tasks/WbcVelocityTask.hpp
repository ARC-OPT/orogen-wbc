/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBCVELOCITYTASK_TASK_HPP
#define WBC_WBCVELOCITYTASK_TASK_HPP

#include "wbc/WbcVelocityTaskBase.hpp"
#include <wbcTypes.hpp>

namespace wbc {

class WbcVelocityScene;

class WbcVelocityTask : public WbcVelocityTaskBase
{
    friend class WbcVelocityTaskBase;
protected:
    base::VectorXd robot_vel;       /** Current robot velocity, converted from joint_state*/
    std::shared_ptr<WbcVelocityScene> wbc_vel_scene;
    HierarchicalLEConstraints constraints_prio;

public:
    WbcVelocityTask(std::string const& name = "wbc::WbcVelocity");
    WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~WbcVelocityTask();

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){WbcVelocityTaskBase::errorHook();}
    void stopHook();
    void cleanupHook();
};
}

#endif

