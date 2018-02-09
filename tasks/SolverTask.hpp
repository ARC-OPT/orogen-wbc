/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_SOLVERTASK_TASK_HPP
#define WBC_SOLVERTASK_TASK_HPP

#include "wbc/SolverTaskBase.hpp"
#include <base/commands/Joints.hpp>

namespace wbc{
    class SolverTask : public SolverTaskBase
    {
	friend class SolverTaskBase;
    protected:    
        base::commands::Joints solver_output;
        base::Time stamp;

        virtual void computeSolverOutput(base::commands::Joints& solver_output) = 0;

    public:
        SolverTask(std::string const& name = "wbc::SolverTask");
        SolverTask(std::string const& name, RTT::ExecutionEngine* engine);
        ~SolverTask();
        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif

