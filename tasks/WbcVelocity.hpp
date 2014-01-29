/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_WBCVELOCITY_TASK_HPP
#define WBC_WBCVELOCITY_TASK_HPP

#include "wbc/WbcVelocityBase.hpp"
#include <wbc/WbcTypes.hpp>

namespace wbc {

class RobotModel;
class HierarchicalWDLSSolver;

class WbcVelocity : public WbcVelocityBase
{
    friend class WbcVelocityBase;
protected:
    HierarchicalWDLSSolver* solver_;
    RobotModel *robot_model_;

    std::vector<SubTaskConfig> wbc_config_;

    std::vector< RTT::InputPort<base::samples::RigidBodyState>* > cart_ref_ports_;
    std::vector< RTT::InputPort<base::samples::Joints>* > jnt_ref_ports_;
    std::vector< RTT::InputPort<base::VectorXd>* > weight_ports_;

    std::vector<base::samples::RigidBodyState> cart_ref_in_;
    std::vector<base::samples::Joints> jnt_ref_in_;
    std::vector<base::VectorXd> weight_in_;

public:
    WbcVelocity(std::string const& name = "wbc::WbcVelocity");
    WbcVelocity(std::string const& name, RTT::ExecutionEngine* engine);
    ~WbcVelocity();

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){WbcVelocityBase::errorHook();}
    void stopHook(){WbcVelocityBase::stopHook();}
    void cleanupHook();
};
}

#endif

