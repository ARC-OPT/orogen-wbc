/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_HIERARCHICALWDLSSOLVERTASK_TASK_HPP
#define WBC_HIERARCHICALWDLSSOLVERTASK_TASK_HPP

#include "wbc/HierarchicalWDLSSolverTaskBase.hpp"
#include <wbc/HierarchicalWDLSSolver.hpp>

namespace wbc {

class HierarchicalWDLSSolverTask : public HierarchicalWDLSSolverTaskBase
{
    friend class HierarchicalWDLSSolverTaskBase;
protected:

    HierarchicalWDLSSolver* solver_;
    std::vector<SolverInput> solver_input_;
    std::vector<PriorityData> priority_data_;
    Eigen::VectorXd x_;
    base::commands::Joints ctrl_out_;
    base::Time stamp_;
    uint nx_;
    bool debug_;

public:
    HierarchicalWDLSSolverTask(std::string const& name = "wbc::HierarchicalWDLSSolverTask");
    HierarchicalWDLSSolverTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~HierarchicalWDLSSolverTask();

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

