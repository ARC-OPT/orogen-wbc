/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_HIERARCHICALWDLSSOLVERTASK_TASK_HPP
#define WBC_HIERARCHICALWDLSSOLVERTASK_TASK_HPP

#include "wbc/HierarchicalWDLSSolverTaskBase.hpp"
#include <wbc/HierarchicalWDLSSolver.hpp>
#include <wbc/SolverTypes.hpp>

namespace wbc {

class HierarchicalWDLSSolverTask : public HierarchicalWDLSSolverTaskBase
{
    friend class HierarchicalWDLSSolverTaskBase;
protected:

    HierarchicalWDLSSolver* solver_;

    std::vector<LinearEqnSystem> linear_eqn_pp_; /** Linear equations sorted by priorities*/
    Eigen::VectorXd x_;
    bool solver_configured_;
    base::VectorXd damping_, inv_condition_numbers_, manipulability_;
    std::vector<base::VectorXd> singular_values_;

    void configureSolver()
    {
        uint n_prios = linear_eqn_pp_.size();
        uint nx = linear_eqn_pp_[0].W_col.size();

        x_.resize(nx);
        singular_values_.resize(n_prios);
        inv_condition_numbers_.resize(n_prios);
        damping_.resize(n_prios);
        manipulability_.resize(n_prios);

        std::vector<int> n_rows_per_prio(n_prios);
        for(uint prio = 0; prio < n_prios; prio++)
        {
            n_rows_per_prio[prio] = linear_eqn_pp_[prio].y_ref.size();
            singular_values_[prio].resize(nx);
        }

        if(!solver_->configure(n_rows_per_prio, nx))
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

