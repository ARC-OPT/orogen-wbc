/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcVelocityTask.hpp"
#include "wbcTypes.hpp"
#include <base/logging.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <wbc/robot_models/KinematicRobotModelKDL.hpp>
#include <wbc/WbcVelocity.hpp>
#include <wbc/solvers/HierarchicalLeastSquaresSolver.hpp>


using namespace wbc;
using namespace std;

WbcVelocityTask::WbcVelocityTask(std::string const& name)
    : WbcVelocityTaskBase(name){
}

WbcVelocityTask::WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcVelocityTaskBase(name, engine){
}
bool WbcVelocityTask::configureHook(){

    wbc = new WbcVelocity();
    robot_model = new KinematicRobotModelKDL(_base_frame.get());
    solver = new HierarchicalLeastSquaresSolver();

    if (! WbcVelocityTaskBase::configureHook())
        return false;

    joint_weights = _initial_joint_weights.get();
    compute_debug = _compute_debug.get();

    // Configure solver
    ((HierarchicalLeastSquaresSolver*)solver)->setMaxSolverOutputNorm(_norm_max.get());
    ((HierarchicalLeastSquaresSolver*)solver)->setMinEigenvalue(_epsilon.get());
    if(_max_solver_output.get().size() > 0)
        ((HierarchicalLeastSquaresSolver*)solver)->setMaxSolverOutput(_max_solver_output.get());

    if(joint_weights.size() != robot_model->getJointNames().size()){
        LOG_ERROR("Number of configured joints is %i, but initial joint weights vector has size %i",  robot_model->getJointNames().size(), joint_weights.size());
        return false;
    }

    robot_vel.resize(robot_model->getJointNames().size());
    robot_vel.setZero();

    uint n_prios = wbc->getConstraints().size();
    singular_values.resize(n_prios);
    inv_condition_numbers.resize(n_prios);
    damping.resize(n_prios);
    manipulability.resize(n_prios);

    return true;
}

bool WbcVelocityTask::startHook(){
    if (! WbcVelocityTaskBase::startHook())
        return false;
    return true;
}

void WbcVelocityTask::updateHook(){

    if(_joint_weights.readNewest(joint_weights) == RTT::NewData){
        for(uint i = 0; i < wbc->getNumberOfPriorities(); i++)
            ((HierarchicalLeastSquaresSolver*)solver)->setJointWeights(joint_weights, i);
    }

    WbcVelocityTaskBase::updateHook();

    for(uint i = 0; i < ctrl_out.size(); i++)
        ctrl_out[i].speed = solver_output(i);

    ctrl_out.time = base::Time::now();
    _ctrl_out.write(ctrl_out);
    _current_joint_weights.write(joint_weights);

    //Compute debug data
    if(state() == RUNNING){
        for(uint i = 0; i <ctrl_out.size(); i++)
            robot_vel(i) = joint_state.getElementByName(ctrl_out.names[i]).speed;
        ((WbcVelocity*)wbc)->evaluateConstraints(solver_output, robot_vel);
    }


    /*
    if(compute_debug)
    {
        /*for(uint prio = 0; prio < equations_.size(); prio++) // Loop priorities
        {
            damping[prio] = solver_.getPriorityData(prio).damping;
            singular_values[prio] = solver_.getPriorityData(prio).singular_values;

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

        _inv_condition_number_pp.write(inv_condition_numbers_);
        _damping_pp.write(damping_);
        //_singular_values_pp.write(singular_values_);
        _manipulability_pp.write(manipulability_);
    }*/
}


void WbcVelocityTask::stopHook(){

    //Set speed to zero
    for(uint i = 0; i < ctrl_out.size(); i++)
        ctrl_out[i].speed = 0.0;
    _ctrl_out.write(ctrl_out);

    WbcVelocityTaskBase::stopHook();
}

void WbcVelocityTask::cleanupHook()
{
    WbcVelocityTaskBase::cleanupHook();

    delete wbc;
    delete solver;
    delete robot_model;
}
