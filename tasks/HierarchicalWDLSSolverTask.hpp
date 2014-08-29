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
    std::vector<ConstraintsPerPrio> constraints_;
    std::vector<std::string> joint_names_;
    Eigen::VectorXd x_;
    base::commands::Joints ctrl_out_;
    base::VectorXd Wx_;
    uint nx_;
    std::vector<Eigen::MatrixXd> A_;
    std::vector<Eigen::VectorXd> Wy_;
    std::vector<Eigen::VectorXd> y_ref_;
    base::VectorXd damping_factors_;
    base::VectorXd condition_numbers_;
    std::vector< base::VectorXd > singular_values_;

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

