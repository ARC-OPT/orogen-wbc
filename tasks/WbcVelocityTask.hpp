/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBCVELOCITYTASK_TASK_HPP
#define WBC_WBCVELOCITYTASK_TASK_HPP

#include "wbc/WbcVelocityTaskBase.hpp"
#include "wbcTypes.hpp"
#include <kdl_conversions/KDLConversions.hpp>
#include <base/logging.h>
#include "ConstraintInterface.hpp"
#include <wbc/WbcVelocity.hpp>

namespace wbc {

class WbcVelocity;

class WbcVelocityTask : public WbcVelocityTaskBase
{
    friend class WbcVelocityTaskBase;
protected:
    WbcVelocity *wbc_;

    ConstraintInterfaceMap constraint_interface_map_;
    std::vector<ConstraintsPerPrio> constraints_;
    std::vector<TaskFrame> task_frames_;
    SolverInput solver_input_;
    base::VectorXd joint_weights_;
    base::commands::Joints ctrl_out_;
    base::samples::Joints joint_state_;
    base::VectorXd solver_output_;
    base::VectorXd robot_vel_;

    void handleSolverOutput()
    {
        if(solver_output_.size() != wbc_->noOfJoints()){
            LOG_ERROR("Solver output size should be %i but is %i", wbc_->noOfJoints(), solver_output_.size());
            throw std::invalid_argument("Invalid solver output");
        }
        for(uint i = 0; i < ctrl_out_.size(); i++)
            ctrl_out_[i].speed = solver_output_(i);
        _ctrl_out.write(ctrl_out_);
    }

    void prepareSolverInput()
    {
        //insert constraint equation into equation system of current priority
        for(uint prio  = 0; prio < constraints_.size(); prio++)
        {
            uint row_index = 0;
            for(uint i = 0; i < constraints_[prio].size(); i++)
            {
                const Constraint& constraint = constraints_[prio][i];
                const uint n_vars = constraint.no_variables;

                //insert constraint equation into equation system of current priority
                solver_input_.priorities[prio].row_weights.segment(row_index, n_vars) = constraint.weights * constraint.activation * (!constraint.constraint_timed_out);
                solver_input_.priorities[prio].system_matrix.block(row_index, 0, n_vars, wbc_->noOfJoints()) = constraint.A;
                solver_input_.priorities[prio].system_input.segment(row_index, n_vars) = constraint.y_ref;

                row_index += n_vars;

                constraints_[prio][i].y_solution = constraints_[prio][i].A * solver_output_;
            }

        }
    }

    void computeConstraintSatisfaction(){
        for(uint prio  = 0; prio < constraints_.size(); prio++)
        {
            for(uint i = 0; i < constraints_[prio].size(); i++)
            {
                constraints_[prio][i].y_solution = constraints_[prio][i].A * solver_output_;
                constraints_[prio][i].y = constraints_[prio][i].A * robot_vel_;
            }
        }
    }

public:
    WbcVelocityTask(std::string const& name = "wbc::WbcVelocity");
    WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~WbcVelocityTask(){}

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){WbcVelocityTaskBase::errorHook();}
    void stopHook(){WbcVelocityTaskBase::stopHook();}
    void cleanupHook();
};
}

#endif

