/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_RobotModelKDLTask_TASK_HPP
#define WBC_RobotModelKDLTask_TASK_HPP

#include "wbc/RobotModelKDLTaskBase.hpp"
#include <wbc/RobotModelKDL.hpp>
#include <wbc/TaskFrame.hpp>

namespace wbc{

class RobotModelKDLTask : public RobotModelKDLTaskBase
{
    friend class RobotModelKDLTaskBase;

protected:
    bool addTaskFrame(::std::string const & id);

    RobotModelKDL* robot_model_;
    base::samples::Joints joint_state_;
    std::vector<TaskFrame> tf_vector_;

public:
    RobotModelKDLTask(std::string const& name = "wbc::RobotModelKDLTask");
    RobotModelKDLTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~RobotModelKDLTask();

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

