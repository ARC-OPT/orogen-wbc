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
    debug_ = _debug.get();
    solver_->setComputeDebug(debug_);

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

    if(_solver_input.read(solver_input_) == RTT::NewData){

        base::Time start = base::Time::now();

        if(!stamp_.isNull())
            _actual_cycle_time.write((base::Time::now() - stamp_).toSeconds());
        stamp_ = base::Time::now();

        if(!solver_->configured())
        {
            std::vector<uint> ny_per_prio;

            for(uint prio  = 0; prio < solver_input_.priorities.size(); prio++)
                ny_per_prio.push_back(solver_input_.priorities[prio].y_ref.size());

            nx_ = solver_input_.joint_names.size();
            x_.resize(nx_);
            x_.setZero();
            ctrl_out_.resize(nx_);
            ctrl_out_.names = solver_input_.joint_names;

            if(!solver_->configure(ny_per_prio, nx_))
                throw std::invalid_argument("Online configuration of HierarchicalWDLSSolver failed");

            solver_->setJointWeights(_initial_joint_weights.get());

            LOG_INFO("Succesfully configured HierarchicalWDLSSolver");
            LOG_INFO("Priorities: ");
            for(uint i = 0; i < ny_per_prio.size(); i++)
                LOG_INFO("Prio: %i, ny: %i", i, ny_per_prio[i]);
        }

        if(_joint_weights.read(joint_weights_) == RTT::NewData)
            solver_->setJointWeights(joint_weights_);
        solver_->getJointWeights((Eigen::VectorXd& )joint_weights_);
        _current_joint_weights.write(joint_weights_);

        solver_->solve(solver_input_, x_);
        for(uint i = 0; i < nx_; i++)
            ctrl_out_[i].speed = x_(i);
        _ctrl_out.write(ctrl_out_);

        if(debug_){
            solver_->getPrioDebugData(priority_data_);
            _priority_data.write(priority_data_);
        }

        _actual_computation_time.write((base::Time::now() - start).toSeconds());
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
}
