/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_ROBOTMODELKDL_TASK_HPP
#define WBC_ROBOTMODELKDL_TASK_HPP

#include "wbc/RobotModelKDLBase.hpp"
#include <kdl/tree.hpp>
#include <wbc/TaskFrame.hpp>
#include <kdl_conversions/KDLConversions.hpp>

namespace wbc{

class TaskFrameInterface{
public:
    TaskFrame* tf;
    RTT::OutputPort<base::samples::RigidBodyState>* pose_out_port;
    RTT::OutputPort<base::MatrixXd>* jac_out_port;
    base::MatrixXd jac_eigen;
    base::samples::RigidBodyState pose_rbs;

    TaskFrameInterface(const KDL::Chain& chain, const uint no_robot_joints, std::map<std::string, int> joint_index_map){
        tf = new TaskFrame(chain, no_robot_joints, joint_index_map);
        pose_out_port = new RTT::OutputPort<base::samples::RigidBodyState>();
        jac_out_port = new RTT::OutputPort<base::MatrixXd>();
        jac_eigen.resize(6,no_robot_joints);
    }
    ~TaskFrameInterface(){
        delete tf;
        delete pose_out_port;
        delete jac_out_port;
    }
    void update(const base::samples::Joints& joint_state){
        tf->update(joint_state);
        kdl_conversions::KDL2RigidBodyState(tf->pose_, pose_rbs);
        pose_rbs.time = base::Time::now();
        pose_out_port->write(pose_rbs);
        jac_eigen = tf->jac_robot_.data;
        jac_out_port->write(jac_eigen);
    }
};

typedef std::map<std::string, int> JointIndexMap;
typedef std::map<std::string, TaskFrameInterface*> TaskFrameInterfaceMap;

class RobotModelKDL : public RobotModelKDLBase
{
    friend class RobotModelKDLBase;
protected:
    /**
     * Add a task frame to the robot model. ID has to be a link in the URDF model. This will autogenerate the corresponding ports. Returns true in case of success, otherwise false.
     */
    virtual bool addTaskFrame(::std::string const & id);

    KDL::Tree tree_;
    TaskFrameInterfaceMap tf_map_;
    JointIndexMap joint_index_map_;
    uint no_robot_joints_;
    base::samples::Joints joint_state_;
    base::Time stamp_;

public:
    RobotModelKDL(std::string const& name = "wbc::RobotModelKDL");
    RobotModelKDL(std::string const& name, RTT::ExecutionEngine* engine);
    ~RobotModelKDL();

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

