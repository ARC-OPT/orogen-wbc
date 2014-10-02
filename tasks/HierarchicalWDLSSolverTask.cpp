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
            singular_values_[prio] = solver_->getPriorityData(prio).singular_values_;
            damping_[prio] = solver_->getPriorityData(prio).damping_;

            double max_s_val = singular_values_[prio].maxCoeff();
            double min_s_val = singular_values_[prio].minCoeff();
            condition_numbers_[prio] = singular_values_[prio].minCoeff() == 0 ? base::infinity<double>() : max_s_val / min_s_val;
        }

        _solver_output.write(x_);
        _computation_time.write((base::Time::now() - start).toSeconds());
        _condition_number.write(condition_numbers_);
        _damping.write(damping_);
        _singular_values.write(singular_values_);
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
