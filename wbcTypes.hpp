#ifndef wbc_TYPES_HPP
#define wbc_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/Eigen.hpp>
#include <wbc/ConstraintConfig.hpp>
#include <base/float.h>

namespace wbc {

struct SubChainConfig{
    std::string root;
    std::string tip;
};

struct SolverInputPerPrio{

    base::MatrixXd system_matrix;
    base::VectorXd system_input;
    base::VectorXd row_weights;

    void doResize(const uint ny, const uint nx)
    {
        system_matrix.resize(ny, nx);
        system_input.resize(ny);
        row_weights.resize(ny);
    }
};

struct SolverInput{
    SolverInput(){}

    void doResize(const std::vector<int>& ny_per_prio, const uint nx)
    {
        column_weights.resize(nx);
        priorities.resize(ny_per_prio.size());
        for(uint i = 0; i < ny_per_prio.size(); i++)
            priorities[i].doResize(ny_per_prio[i], nx);
    }

    std::vector<SolverInputPerPrio> priorities;
    base::VectorXd column_weights;
};

struct ConstraintResult{
    ConstraintResult(){}
    ConstraintResult(const ConstraintConfig& conf){
        config = conf;
        uint no_constraints;
        if(conf.type == cart)
            no_constraints = 6;
        else
            no_constraints = conf.joint_names.size();
        actual_task_weights.resize(no_constraints);
        actual_task_weights.setConstant(base::NaN<double>());
        y_ref.resize(no_constraints);
        y_ref.setConstant(base::NaN<double>());
        y_solution.resize(no_constraints);
        y_solution.setConstant(base::NaN<double>());
        y.resize(no_constraints);
        y.setConstant(base::NaN<double>());
    }

    /** Constraint configuration copied from whole body controller*/
    ConstraintConfig config;

    /** Task weights actually used in the solver: actual_task_weights = activation * (!timeout) * task_weights */
    base::VectorXd actual_task_weights;

    /** Reference input for this constraint*/
    base::VectorXd y_ref;

    /** Solution returned by the solver for this constraint*/
    base::VectorXd y_solution;

    /** Executed constraint by the robot*/
    base::VectorXd y;

};

struct ConstraintResultPerPrio{

    /** Constraints on this priority level*/
    std::vector<ConstraintResult> constraints;

    /** Singular values of the task Jacobian of this priority*/
    base::VectorXd singular_values;

    /** Condition number: max(singular_values) / min(singular_values) */
    double condition_number;

    /** Damping factor used for matrix inversion on this priority level*/
    double damping;
};
}

#endif

