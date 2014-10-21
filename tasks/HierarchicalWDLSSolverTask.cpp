/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "HierarchicalWDLSSolverTask.hpp"

using namespace wbc;

HierarchicalWDLSSolverTask::HierarchicalWDLSSolverTask(std::string const& name)
    : HierarchicalWDLSSolverTaskBase(name)
{
}

HierarchicalWDLSSolverTask::HierarchicalWDLSSolverTask(std::string const& name, RTT::ExecutionEngine* engine)
    : HierarchicalWDLSSolverTaskBase(name, engine)
{
}

HierarchicalWDLSSolverTask::~HierarchicalWDLSSolverTask()
{
}

bool HierarchicalWDLSSolverTask::configureHook()
{
    if (! HierarchicalWDLSSolverTaskBase::configureHook())
        return false;

    solver_ = new HierarchicalWDLSSolver();
    solver_->setNormMax(_norm_max.get());
    solver_->setSVDMethod(_svd_method.get());
    solver_->setEpsilon(_epsilon.get());

    return true;
}

bool HierarchicalWDLSSolverTask::startHook()
{
    if (! HierarchicalWDLSSolverTaskBase::startHook())
        return false;
    solver_configured_ = false;
    return true;
}

void HierarchicalWDLSSolverTask::updateHook()
{
    HierarchicalWDLSSolverTaskBase::updateHook();

    int ret = _linear_eqn_pp.read(linear_eqn_pp_);
    if(ret == RTT::NoData)
    {
        if(state() != WAITING_FOR_EQN_SYSTEM)
            state(WAITING_FOR_EQN_SYSTEM);
        return;
    }

    if(state() != RUNNING)
        state(RUNNING);

    if(ret == RTT::NewData)
    {
        if(linear_eqn_pp_.empty())
            throw std::invalid_argument("Empty equation system");

        if(!solver_configured_)
            configureSolver();

        base::Time start = base::Time::now();

        solver_->solve(linear_eqn_pp_, x_);

        for(uint prio = 0; prio < linear_eqn_pp_.size(); prio++)
        {
            damping_[prio] = solver_->getPriorityData(prio).damping_;
            singular_values_[prio] = solver_->getPriorityData(prio).singular_values_;

            //Find min and max singular value. Since some singular values might be zero due to deactivated
            //constraints (zero row weight). Only consider the singular values, which correspond to rows with non-zero weights

            double max_s_val = singular_values_[prio].maxCoeff();
            double min_s_val = base::infinity<double>();
            manipulability_[prio] = 1;
            for(uint i = 0; i < singular_values_[prio].size(); i++)
            {
                if(singular_values_[prio](i) < min_s_val && singular_values_[prio](i) > 1e-5)
                {
                    min_s_val = singular_values_[prio](i);
                    manipulability_[prio] *= singular_values_[prio](i);
                }
            }
            //If all row weights are zero, inverse condition number should be 0
            if(min_s_val == base::infinity<double>())
                min_s_val = 0;
            if(manipulability_[prio] == 1)
                manipulability_[prio] = 0;

            inv_condition_numbers_[prio] = min_s_val / max_s_val;
        }

        _solver_output.write(x_);
        _computation_time.write((base::Time::now() - start).toSeconds());
        _inv_condition_number_pp.write(inv_condition_numbers_);
        _damping_pp.write(damping_);
        _singular_values_pp.write(singular_values_);
        _manipulability_pp.write(manipulability_);
    }
}

void HierarchicalWDLSSolverTask::errorHook()
{
    HierarchicalWDLSSolverTaskBase::errorHook();
}

void HierarchicalWDLSSolverTask::stopHook()
{
    HierarchicalWDLSSolverTaskBase::stopHook();
}

void HierarchicalWDLSSolverTask::cleanupHook()
{
    HierarchicalWDLSSolverTaskBase::cleanupHook();

    delete solver_;
}
