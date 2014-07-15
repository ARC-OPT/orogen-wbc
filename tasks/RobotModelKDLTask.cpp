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

    robot_model_ = new RobotModelKDL(tree);
    std::vector<std::string> task_frame_ids = _task_frame_ids.get();
    for(uint i =0 ; i < task_frame_ids.size(); i++)
    {
        if(!addTaskFrame(task_frame_ids[i]))
            return false;
    }

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

        for(uint i = 0; i < tf_vector_.size(); i++){
            TfKDLToTf(*robot_model_->getTaskFrame(tf_vector_[i].tf_name), tf_vector_[i]);
            tf_vector_[i].time = tf_vector_[i].pose.time = base::Time::now();
        }
        _task_frames.write(tf_vector_);
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
    robot_model_ = 0;
    tf_vector_.clear();
}

bool RobotModelKDLTask::addTaskFrame(const std::string &id){

    if(robot_model_)
    {
        if(!robot_model_->hasTaskFrame(id))
        {
            if(!robot_model_->addTaskFrame(id))
                return false;

            TaskFrame tf;
            TfKDLToTf(*robot_model_->getTaskFrame(id), tf);
            tf.pose.sourceFrame = robot_model_->robotRoot();
            tf.pose.targetFrame = id;
            tf_vector_.push_back(tf);
        }
    }
    else
    {
        LOG_ERROR("Call to addTaskFrame failed. Is the component configured?");
        throw std::runtime_error("Invalid call to addTaskFrame");
    }
    return true;
}
