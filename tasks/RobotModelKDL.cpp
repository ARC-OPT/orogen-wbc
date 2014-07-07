/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RobotModelKDL.hpp"
#include <kdl_parser/kdl_parser.hpp>

using namespace wbc;

RobotModelKDL::RobotModelKDL(std::string const& name)
    : RobotModelKDLBase(name)
{
}

RobotModelKDL::RobotModelKDL(std::string const& name, RTT::ExecutionEngine* engine)
    : RobotModelKDLBase(name, engine)
{
}

RobotModelKDL::~RobotModelKDL()
{
}

bool RobotModelKDL::configureHook()
{
    if (! RobotModelKDLBase::configureHook())
        return false;

    //
    // Load URDF Model
    //

    KDL::Tree full_tree;
    if(!kdl_parser::treeFromFile(_urdf.get(), full_tree)){
        LOG_ERROR("Unable to parse KDL Tree from URDF file %s", _urdf.get().c_str());
        return false;
    }
    std::vector<wbc::SubChainConfig> reduced_tree = _reduced_tree.get();
    if(reduced_tree.empty())
        tree_ = full_tree;
    else
    {
        for(uint i = 0; i < reduced_tree.size(); i++)
        {
            KDL::Chain chain;
            if(!full_tree.getChain(reduced_tree[i].root, reduced_tree[i].tip, chain))
            {
                LOG_ERROR("Could not extract sub chain between %s and %s from KDL tree", reduced_tree[i].root.c_str(), reduced_tree[i].tip.c_str());
                return false;
            }
            //Root of first subchain will be root of whole KDL tree
            if(i == 0)
                tree_.addSegment(KDL::Segment(reduced_tree[i].root, KDL::Joint(reduced_tree[i].root,KDL::Joint::None),KDL::Frame::Identity()), "root");
            tree_.addChain(chain, reduced_tree[i].root);
        }
    }

    no_robot_joints_ = tree_.getNrOfJoints();

    //
    // Create joint index map
    //

    // The joint name property can define the internal order of joints.
    // If no joint names are given, the order will be the same as in the KDL tree
    std::vector<std::string> joint_names = _joint_names.get();
    if(joint_names.empty())
    {
        KDL::SegmentMap segments = tree_.getSegments();
        uint idx = 0;
        for(KDL::SegmentMap::iterator it = segments.begin(); it != segments.end(); it++)
        {
            KDL::Segment seg = it->second.segment;
            if(seg.getJoint().getType() != KDL::Joint::None)
                joint_index_map_[seg.getJoint().getName()] = idx++;
        }
    }
    else
    {
        for(uint i = 0; i < joint_names.size(); i++)
            joint_index_map_[joint_names[i]] = i;

        //Check if all joints in tree are in joint index map
        KDL::SegmentMap segments = tree_.getSegments();
        for(KDL::SegmentMap::iterator it = segments.begin(); it != segments.end(); it++)
        {
            KDL::Segment seg = it->second.segment;
            if(seg.getJoint().getType() != KDL::Joint::None)
            {
                if(joint_index_map_.count(seg.getJoint().getName()) == 0)
                {
                    LOG_ERROR("Joint with name %s is in KDL::Tree but not in joint names parameter", seg.getJoint().getName().c_str());
                    LOG_ERROR("If the order of joints shall be fixed with the joint names parameter, all joints in tree have to be given here");
                    return false;
                }
            }
        }
    }

    return true;
}

bool RobotModelKDL::startHook()
{
    if (! RobotModelKDLBase::startHook())
        return false;
    return true;
}

void RobotModelKDL::updateHook()
{
    RobotModelKDLBase::updateHook();

    if(_joint_state.read(joint_state_) == RTT::NewData){
        for(TaskFrameInterfaceMap::iterator it = tf_map_.begin(); it != tf_map_.end(); it++){
            it->second->update(joint_state_);
        }
    }
}

void RobotModelKDL::errorHook()
{
    RobotModelKDLBase::errorHook();
}

void RobotModelKDL::stopHook()
{
    RobotModelKDLBase::stopHook();
}

void RobotModelKDL::cleanupHook()
{
    RobotModelKDLBase::cleanupHook();

    for(TaskFrameInterfaceMap::iterator it = tf_map_.begin(); it != tf_map_.end(); it++){
        ports()->removePort(it->second->pose_out_port->getName());
        ports()->removePort(it->second->jac_out_port->getName());
        delete it->second;
    }

    tf_map_.clear();
    joint_index_map_.clear();
}

bool RobotModelKDL::addTaskFrame(const std::string &id){
    if(tf_map_.count(id) == 0)
    {
        KDL::Chain chain;
        std::string robot_root = tree_.getRootSegment()->first;
        if(!tree_.getChain(robot_root, id, chain))
        {
            LOG_ERROR("Could not extract kinematic chain between %s and %s from robot tree", robot_root.c_str(), id.c_str());
            return false;
        }
        TaskFrameInterface* tf_iface = new TaskFrameInterface(chain, no_robot_joints_, joint_index_map_);
        tf_map_[id] = tf_iface;
        ports()->addPort("pose_" + id, *tf_iface->pose_out_port);
        ports()->addPort("jac_" + id, *tf_iface->jac_out_port);

        LOG_DEBUG("Sucessfully added task frame %s", id.c_str());
        LOG_DEBUG("TF Map now contains:");
        for(TaskFrameInterfaceMap::iterator it = tf_map_.begin(); it != tf_map_.end(); it++)
            LOG_DEBUG("%s", it->first.c_str());
        LOG_DEBUG("\n");
        return true;
    }
    else{
        LOG_ERROR("Task Frame with id %s has already been added", id.c_str());
        return false;
    }
}
