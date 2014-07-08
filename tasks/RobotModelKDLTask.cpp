/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RobotModelKDLTask.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <kdl/tree.hpp>

using namespace wbc;

RobotModelKDLTask::RobotModelKDLTask(std::string const& name)
    : RobotModelKDLTaskBase(name)
{
}

RobotModelKDLTask::RobotModelKDLTask(std::string const& name, RTT::ExecutionEngine* engine)
    : RobotModelKDLTaskBase(name, engine)
{
}

RobotModelKDLTask::~RobotModelKDLTask()
{
}

bool RobotModelKDLTask::configureHook()
{
    if (! RobotModelKDLTaskBase::configureHook())
        return false;

    // Load URDF Model:
    KDL::Tree full_tree, tree;
    if(!kdl_parser::treeFromFile(_urdf.get(), full_tree))
    {
        LOG_ERROR("Unable to parse KDL Tree from URDF file %s", _urdf.get().c_str());
        return false;
    }

    //Construct tree, if no reduced tree is given, use full tree
    std::vector<wbc::SubChainConfig> reduced_tree = _reduced_tree.get();
    if(reduced_tree.empty())
        tree = full_tree;
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
                tree.addSegment(KDL::Segment(reduced_tree[i].root, KDL::Joint(reduced_tree[i].root,KDL::Joint::None),KDL::Frame::Identity()), "root");
            tree.addChain(chain, reduced_tree[i].root);
        }
    }

    robot_model_ = new RobotModelKDL(tree, _joint_names.get());

    return true;
}

bool RobotModelKDLTask::startHook()
{
    if (! RobotModelKDLTaskBase::startHook())
        return false;
    return true;
}

void RobotModelKDLTask::updateHook()
{
    RobotModelKDLTaskBase::updateHook();

    if(_joint_state.read(joint_state_) == RTT::NewData){

        if(!stamp_.isNull())
            _actual_cycle_time.write((base::Time::now() - stamp_).toSeconds());
        stamp_ = base::Time::now();

        robot_model_->update(joint_state_);

        for(TaskFramePortMap::iterator it = tf_port_map_.begin(); it != tf_port_map_.end(); it++)
        {
            TaskFrameKDL* tf_kdl = robot_model_->getTaskFrame(it->first);
            tf_map_[it->first].jac = tf_kdl->jac_robot_kdl_.data;
            kdl_conversions(tf_kdl->pose_kdl_, tf_map_[it->first].pose);
            it->second->write(tf_map[it->first]);
        }
    }
}

void RobotModelKDLTask::errorHook(){
    RobotModelKDLTaskBase::errorHook();
}

void RobotModelKDLTask::stopHook(){
    RobotModelKDLTaskBase::stopHook();
}

void RobotModelKDLTask::cleanupHook(){
    RobotModelKDLTaskBase::cleanupHook();

    delete robot_model_;

    for(TaskFramePortMap::iterator it = tf_port_map_.begin(); it != tf_port_map_.end(); it++)
    {
        ports()->removePort(it->second->getName());
        delete it->second;
    }
    tf_port_map_.clear();
    tf_map_.clear();
}

bool RobotModelKDLTask::addTaskFrame(const std::string &id){

    if(state() != STOPPED)
    {
        LOG_ERROR("Call to addTaskFrame only allowed in state STOPPED, current state is %i", state());
        throw std::runtime_error("Invalid call to addTaskFrame");
    }

    if(tf_port_map_.count(id) == 0)
    {
        RTT::OutputPort<TaskFrame>* port = new RTT::OutputPort<TaskFrame>("tf_" + id);
        ports()->addPort("tf_" + id, *port);
        tf_port_map_[id] = port;
        tf_map_[id] = TaskFrame(robot_model_->getNoOfJoints(), id);
    }
    return robot_model_->addTaskFrame(id);
}
