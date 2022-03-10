/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBCACCELERATIONTASK_TASK_HPP
#define WBC_WBCACCELERATIONTASK_TASK_HPP

#include "wbc/WbcAccelerationTaskBase.hpp"
#include <base/samples/Wrenches.hpp>

namespace wbc{

/**
 * @brief Acceleration based implementation of the WBC Scene.
 */
class WbcAccelerationTask : public WbcAccelerationTaskBase
{
    friend class WbcAccelerationTaskBase;
protected:
public:
    WbcAccelerationTask(std::string const& name = "wbc::WbcAccelerationTask");
    WbcAccelerationTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~WbcAccelerationTask();

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};

}

#endif

