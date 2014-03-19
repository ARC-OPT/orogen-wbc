/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBCVELOCITYTASK_TASK_HPP
#define WBC_WBCVELOCITYTASK_TASK_HPP

#include "wbc/WbcVelocityTaskBase.hpp"
#include <wbc/wbcTypes.hpp>
#include <wbc/HierarchicalWDLSSolver.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <wbc/WbcVelocity.hpp>
#include <base/logging.h>

namespace wbc {

class SubTaskInterface
{
public:
    SubTaskInterface(const SubTaskConfig& conf);
    ~SubTaskInterface();

    void resetTimeout();
    void update();

    std::string task_name;
    SubTaskConfig config;

    RTT::InputPort<base::samples::RigidBodyState>* cart_ref_port;
    RTT::InputPort<base::samples::Joints>* jnt_ref_port;
    RTT::InputPort<base::VectorXd>* weight_port;
    RTT::InputPort<double>* activation_port;

    //Debug Ports
    RTT::OutputPort<base::VectorXd>* y_solution_out_port;
    RTT::OutputPort<base::VectorXd>* y_act_out_port;
    RTT::OutputPort<base::MatrixXd>* A_task_out_port;
    RTT::OutputPort<base::samples::RigidBodyState>* pose_out_port;

    base::samples::RigidBodyState cart_ref; /** Cartesian Reference values */
    base::samples::Joints jnt_ref;          /** Jnt reference values */
    base::VectorXd y_des;                   /** Generic joint reference */
    base::VectorXd weights;                 /** Task weights */
    double activation;                      /** Activation function. Will be pre-multiplied with the task weights */
    base::Time last_task_input;             /** Last time new reference values arrived */
    int task_timed_out_;                    /** Is this task timed out?*/
};
typedef std::map< std::string, SubTaskInterface* > SubTaskInterfaceMap;


class WbcVelocityTask : public WbcVelocityTaskBase
{
    friend class WbcVelocityTaskBase;
protected:
    HierarchicalWDLSSolver solver_;
    WbcVelocity wbc_;

    SubTaskInterfaceMap sub_task_interface_map_;

    base::VectorXd joint_weights_;
    base::MatrixXd joint_weight_mat_;
    base::VectorXd solver_output_, act_robot_velocity_;
    base::samples::Joints ctrl_out_;
    base::samples::Joints joint_status_;
    bool write_debug_;
    double task_timeout_;

    void addPortsForSubTask(const SubTaskInterface* sti)
    {
        if(sti->config.type == wbc::task_type_cartesian){
            ports()->addPort(sti->cart_ref_port->getName(), *(sti->cart_ref_port));
            ports()->addPort(sti->pose_out_port->getName(), *(sti->pose_out_port));
        }
        else
            ports()->addPort((sti->jnt_ref_port)->getName(), *(sti->jnt_ref_port));

        ports()->addPort(sti->weight_port->getName(), *(sti->weight_port));
        ports()->addPort(sti->activation_port->getName(), *(sti->activation_port));

        if(write_debug_){
            ports()->addPort(sti->y_solution_out_port->getName(), *(sti->y_solution_out_port));
            ports()->addPort(sti->y_act_out_port->getName(), *(sti->y_act_out_port));
            ports()->addPort(sti->A_task_out_port->getName(), *(sti->A_task_out_port));
        }
    }

    void removePortsOfSubTask(const SubTaskInterface* sti)
    {
        ports()->removePort(sti->weight_port->getName());
        ports()->removePort(sti->activation_port->getName());

        if(ports()->getPort(sti->y_solution_out_port->getName())){
            ports()->removePort(sti->y_solution_out_port->getName());
        }
        if(ports()->getPort(sti->y_act_out_port->getName())){
            ports()->removePort(sti->y_act_out_port->getName());
        }
        if(ports()->getPort(sti->A_task_out_port->getName())){
            ports()->removePort(sti->A_task_out_port->getName());
        }
        if(ports()->getPort(sti->pose_out_port->getName())){
            ports()->removePort(sti->pose_out_port->getName());
        }
        if(ports()->getPort(sti->cart_ref_port->getName())){
            ports()->removePort(sti->cart_ref_port->getName());
        }
        if(ports()->getPort(sti->jnt_ref_port->getName())){
            ports()->removePort(sti->jnt_ref_port->getName());
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

