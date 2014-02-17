/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBCVELOCITYTASK_TASK_HPP
#define WBC_WBCVELOCITYTASK_TASK_HPP

#include "wbc/WbcVelocityTaskBase.hpp"
#include <wbc/wbcTypes.hpp>
#include <wbc/HierarchicalWDLSSolver.hpp>
#include <wbc/WbcVelocity.hpp>

namespace wbc {

typedef std::map< std::string, RTT::InputPort<base::samples::RigidBodyState>* > CartPortMap;
typedef std::map< std::string, RTT::OutputPort<base::samples::RigidBodyState>* > CartOutPortMap;
typedef std::map< std::string, RTT::InputPort<base::samples::Joints>* > JntPortMap;
typedef std::map< std::string, RTT::InputPort<base::VectorXd>* > WeightPortMap;
typedef std::map< std::string, RTT::OutputPort<base::VectorXd>* > YOutPortMap;
typedef std::map< std::string, RTT::OutputPort<base::MatrixXd>* > AOutPortMap;

class WbcVelocityTask : public WbcVelocityTaskBase
{
    friend class WbcVelocityTaskBase;
protected:
    HierarchicalWDLSSolver solver_;
    WbcVelocity wbc_;

    base::samples::Joints joint_status_;
    CartPortMap cart_ref_ports_;
    JntPortMap jnt_ref_ports_;
    WeightPortMap weight_ports_;

    std::map<std::string, base::samples::RigidBodyState> cart_ref_in_; /** Cart Reference values */
    std::map<std::string, base::samples::Joints> jnt_ref_in_; /** Jnt reference values */
    std::map<std::string, base::VectorXd> weight_in_; /** Task weights */
    base::VectorXd joint_weights_;
    base::MatrixXd joint_weight_mat_;
    base::VectorXd solver_output_;
    base::samples::Joints ctrl_out_;

    //Debug Ports
    YOutPortMap y_task_out_ports_;
    AOutPortMap A_task_out_ports_;
    CartOutPortMap pose_out_ports_;

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

