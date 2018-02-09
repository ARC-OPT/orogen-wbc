/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_HIERARCHICALLSSOLVERTASK_TASK_HPP
#define WBC_HIERARCHICALLSSOLVERTASK_TASK_HPP

#include "wbc/HierarchicalLSSolverTaskBase.hpp"
#include <wbc/HierarchicalLSSolver.hpp>
#include <wbcTypes.hpp>

namespace wbc{
    class HierarchicalLSSolverTask : public HierarchicalLSSolverTaskBase
    {
	friend class HierarchicalLSSolverTaskBase;
    protected:

        std::shared_ptr<HierarchicalLSSolver> solver;
        HierarchicalLEConstraints constraints_prio;
        base::VectorXd solver_output_raw;
        base::VectorXd joint_weights;                      /** Joint weights of the whole robot*/

        virtual void computeSolverOutput(base::commands::Joints& solver_output);

    public:
        HierarchicalLSSolverTask(std::string const& name = "wbc::HierarchicalLSSolverTask");
        HierarchicalLSSolverTask(std::string const& name, RTT::ExecutionEngine* engine);
        ~HierarchicalLSSolverTask();
        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif

