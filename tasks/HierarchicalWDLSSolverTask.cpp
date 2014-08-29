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
    joint_names_ = _joint_names.get();
    Wx_= _initial_joint_weights.get();

    ctrl_out_.resize(joint_names_.size());
    ctrl_out_.names = joint_names_;
    x_.resize(joint_names_.size());
    x_.setZero();
    nx_ = joint_names_.size();

    return true;
}

bool HierarchicalWDLSSolverTask::startHook()
{
    if (! HierarchicalWDLSSolverTaskBase::startHook())
        return false;
    return true;
}

void HierarchicalWDLSSolverTask::updateHook()
{
    HierarchicalWDLSSolverTaskBase::updateHook();

    if(_constraints.read(constraints_) == RTT::NewData){

        base::Time start = base::Time::now();

        //Configure solver online, if this has not been done yet
        if(!solver_->configured())
        {
            uint n_prios = constraints_.size();

            A_.resize(n_prios);
            Wy_.resize(n_prios);
            y_ref_.resize(n_prios);
            singular_values_.resize(n_prios);
            damping_factors_.resize(n_prios);
            condition_numbers_.resize(n_prios);

            std::vector<uint> ny_per_prio(n_prios, 0); // number of constraint per priority
            for(uint prio  = 0; prio < n_prios; prio++)
            {
                for(uint i = 0; i < constraints_[prio].size(); i++)
                    ny_per_prio[prio] += constraints_[prio][i].y_ref.size();

                A_[prio].resize(ny_per_prio[prio], nx_);
                Wy_[prio].resize(ny_per_prio[prio]);
                y_ref_[prio].resize(ny_per_prio[prio]);
            }


            if(!solver_->configure(ny_per_prio, nx_))
                throw std::invalid_argument("Online configuration of HierarchicalWDLSSolver failed");

            LOG_INFO("Succesfully configured HierarchicalWDLSSolver");
            LOG_INFO("Priorities: ");
            for(uint i = 0; i < ny_per_prio.size(); i++)
                LOG_INFO("Prio: %i, ny: %i", i, ny_per_prio[i]);
        }

        _joint_weights.read(Wx_);

        //insert constraint equation into equation system of current priority
        uint row_index = 0;
        for(uint prio  = 0; prio < constraints_.size(); prio++)
        {
            damping_factors_[prio] = solver_->priorities_[prio].damping_;
            singular_values_[prio] = solver_->priorities_[prio].singular_values_;
            condition_numbers_[prio] = solver_->priorities_[prio].singular_values_(0) /
                    solver_->priorities_[prio].singular_values_(solver_->priorities_[prio].ny_);

            for(uint i = 0; i < constraints_[prio].size(); i++)
            {
                const Constraint& constraint = constraints_[prio][i];
                const uint n_vars = constraint.no_variables;

                Wy_[prio].segment(row_index, n_vars) = constraint.weights * constraint.activation * (!constraint.constraint_timed_out);
                A_[prio].block(row_index, 0, n_vars, nx_) = constraint.A;
                y_ref_[prio].segment(row_index, n_vars) = constraint.y_ref;

                row_index += n_vars;
            }
        }

        solver_->solve(A_, Wy_, y_ref_, Wx_, x_);

        for(uint i = 0; i < nx_; i++)
            ctrl_out_[i].speed = x_(i);

        _condition_numbers.write(condition_numbers_);
        _damping_factors.write(damping_factors_);
        _singular_values.write(singular_values_);
        _ctrl_out.write(ctrl_out_);
        _computation_time.write((base::Time::now() - start).toSeconds());
        _current_joint_weights.write(Wx_);
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
