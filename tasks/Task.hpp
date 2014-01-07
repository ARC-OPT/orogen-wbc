/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_TASK_TASK_HPP
#define WBC_TASK_TASK_HPP

#include "wbc/TaskBase.hpp"
#include <wbc/Wbc.hpp>
#include <yaml-cpp/yaml.h>
#include <srdfdom/model.h>
#include <urdf_model/model.h>

namespace wbc {


class Task : public TaskBase
{
    friend class TaskBase;
protected:
    Wbc *wbc_;

    std::vector< RTT::InputPort<base::VectorXd>* > ref_ports_, weight_ports_;

    WbcInput reference_, task_weights_;
    base::samples::Joints status_;
    base::commands::Joints solver_output_;
    WbcConfig wbc_config_;
    base::VectorXd joint_weights_;
    srdf::Model srdf_model_;

    SubTaskConfig parseSubTaskConfig(const YAML::Node& node);

public:
    Task(std::string const& name = "wbc::Task");
    Task(std::string const& name, RTT::ExecutionEngine* engine);
    ~Task(){}

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){TaskBase::errorHook();}
    void stopHook(){TaskBase::stopHook();}
    void cleanupHook();
};
}

#endif

