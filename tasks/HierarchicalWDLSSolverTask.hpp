/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_HIERARCHICALWDLSSOLVERTASK_TASK_HPP
#define WBC_HIERARCHICALWDLSSOLVERTASK_TASK_HPP

#include "wbc/HierarchicalWDLSSolverTaskBase.hpp"
#include <wbc/HierarchicalWDLSSolver.hpp>
#include "wbcTypes.hpp"

namespace wbc {

class HierarchicalWDLSSolverTask : public HierarchicalWDLSSolverTaskBase
{
    friend class HierarchicalWDLSSolverTaskBase;
protected:

    HierarchicalWDLSSolver* solver_;
    SolverInput solver_input_;
    std::vector<Eigen::MatrixXd> A_;
    std::vector<Eigen::VectorXd> Wy_, y_ref_;
    Eigen::VectorXd x_;
    bool solver_configured_;
    base::VectorXd damping_, condition_numbers_;
    std::vector<base::VectorXd> singular_values_;

    void configureSolver()
    {
        uint n_prios = solver_input_.priorities.size();
        uint nx = solver_input_.column_weights.size();

        if(A_.size() != n_prios)
            A_.resize(n_prios);
        if(y_ref_.size() != n_prios)
            y_ref_.resize(n_prios);
        if(Wy_.size() != n_prios)
            Wy_.resize(n_prios);
        if(x_.size() != solver_input_.column_weights.size())
            x_.resize(solver_input_.column_weights.size());

        singular_values_.resize(n_prios);
        condition_numbers_.resize(n_prios);
        damping_.resize(n_prios);

        std::vector<int> ny_per_prio(n_prios);
        for(uint prio = 0; prio < n_prios; prio++){
            ny_per_prio[prio] = solver_input_.priorities[prio].system_input.size();
            singular_values_[prio].resize(std::min((int)nx, (int)ny_per_prio[prio]));
        }

        if(!solver_->configure(ny_per_prio, nx))
            throw std::runtime_error("Unable to configure solver");

        solver_configured_ = true;
    }

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

