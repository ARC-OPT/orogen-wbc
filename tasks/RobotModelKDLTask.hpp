/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_RobotModelKDLTask_TASK_HPP
#define WBC_RobotModelKDLTask_TASK_HPP

#include "wbc/RobotModelKDLTaskBase.hpp"
#include <wbc/RobotModelKDL.hpp>
#include <wbc/TaskFrame.hpp>

namespace wbc{

typedef std::map<std::string, RTT::OutputPort<TaskFrame>*> TaskFramePortMap;
typedef std::map<std::string, TaskFrame> TaskFrameMap;

class RobotModelKDLTask : public RobotModelKDLTaskBase
{
    friend class RobotModelKDLTaskBase;

protected:
    /**
     * Add a task frame to the robot model. ID has to be a link in the URDF model. This will autogenerate the corresponding ports. Returns true in case of success, otherwise false.
     */
    virtual bool addTaskFrame(::std::string const & id);

    RobotModelKDL* robot_model_;
    base::samples::Joints joint_state_;
    base::Time stamp_;
    TaskFramePortMap tf_port_map_;
    TaskFrameMap tf_map_;

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

