/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include <wbc/SubTask.hpp>
#include "WbcVelocityTask.hpp"
#include <base/logging.h>
#include <wbc/TaskFrame.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <fstream>

using namespace wbc;
using namespace std;

WbcVelocityTask::WbcVelocityTask(std::string const& name)
    : WbcVelocityTaskBase(name)
{
}

WbcVelocityTask::WbcVelocityTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcVelocityTaskBase(name, engine)
{
}

bool WbcVelocityTask::configureHook()
{
    if (! WbcVelocityTaskBase::configureHook())
        return false;

    std::string urdf_file= _urdf.get();
    std::string srdf_file = _srdf.get();
    std::vector<wbc::SubTaskConfigSRDF> wbc_config_srdf = _wbc_config.get();

    //
    // Load urdf model: Kinematics of the robot
    //
    std::ifstream urdf_stream(urdf_file.c_str());
    if(!urdf_stream.is_open()){
        LOG_ERROR("Error opening urdf file %s", urdf_file.c_str());
        return false;
    }
    std::string xml((std::istreambuf_iterator<char>(urdf_stream)), std::istreambuf_iterator<char>());
    boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(xml);
    LOG_DEBUG("Parsing URDF file done");

    //
    // Load SRDF model: Semantic information about joint groups, chains and end effectors
    //
    srdf::Model srdf_model;
    if(!srdf_model.initFile(*urdf_model.get(), srdf_file)){
        LOG_ERROR("Error opening srdf file %s", srdf_file.c_str());
        return false;
    }
    LOG_DEBUG("Parsing SRDF file done");

    //
    // Extend the wbc config by the information from srdf
    //

    //Put joint groups in a map for easier search
    std::vector<srdf::Model::Group> groups = srdf_model.getGroups();
    std::map<std::string, srdf::Model::Group> group_map;
    for(uint i = 0; i < groups.size(); i++)
        group_map[groups[i].name_] = groups[i];

    //Check which is the highest priority
    uint max_prio = 0;
    for(uint i = 0; i < wbc_config_srdf.size(); i++){
        if(wbc_config_srdf[i].priority > max_prio)
            max_prio = wbc_config_srdf[i].priority;
    }

    std::vector<uint> cart_idx(max_prio + 1,0), jnt_idx(max_prio + 1, 0);
    std::vector<SubTaskConfig> wbc_config;
    for(uint i = 0; i < wbc_config_srdf.size(); i++){

        std::string group_name = wbc_config_srdf[i].joint_group;
        if(group_map.count(group_name) == 0){
            LOG_ERROR("Joint group %s does not exist in srdf file", group_name.c_str());
            return false;
        }

        SubTaskConfig conf;
        conf.type = wbc_config_srdf[i].type;
        conf.priority = wbc_config_srdf[i].priority;

        srdf::Model::Group joint_group = group_map[group_name];

        uint no_task_vars;
        switch(conf.type){
        case wbc::task_type_cartesian:{

            if(joint_group.chains_.empty()){
                LOG_ERROR("Task is Cartesian but joint group %s has no chains", group_name.c_str());
                return false;
            }

            if(joint_group.chains_.size() > 1)
                LOG_WARN("No of chains in joint group %s is bigger than 1. Using the first one");

            no_task_vars = 6;
            conf.root = joint_group.chains_[0].first;
            conf.tip = joint_group.chains_[0].second;

            std::stringstream ss;
            ss<<"cart_"<<conf.priority<<"_"<<cart_idx[conf.priority]++;
            conf.name = ss.str().c_str();

            //Add cartesian port
            RTT::InputPort<base::samples::RigidBodyState> *ref_port = new RTT::InputPort<base::samples::RigidBodyState>("/ref_" + conf.name);
            ports()->addPort("ref_" + conf.name, *ref_port);
            cart_ref_ports_[conf.name] = ref_port;

            //Add Debug port: Task Poses
            RTT::OutputPort<base::samples::RigidBodyState>* pose_out_port = new RTT::OutputPort<base::samples::RigidBodyState>("pose_" + conf.name);
            ports()->addPort("pose_" + conf.name, *pose_out_port);
            pose_out_ports_[conf.name] = pose_out_port;

            kdl_conversions::KDL2RigidBodyState(KDL::Twist::Zero(), cart_ref_in_[conf.name]);
            break;
        }
        case wbc::task_type_joint:{
            //Only accept joints from srdf as input for joint space task
            if(joint_group.joints_.empty()){
                std::stringstream ss;
                LOG_ERROR("Task is in joint space but joint group  %s has no joints", group_name.c_str());
                return false;
            }

            conf.joints = joint_group.joints_;
            no_task_vars = joint_group.joints_.size();

            std::stringstream ss;
            ss<<"jnt_"<<conf.priority<<"_"<<jnt_idx[conf.priority]++;
            conf.name = ss.str().c_str();

            //Add joint port
            RTT::InputPort<base::samples::Joints> *ref_port = new RTT::InputPort<base::samples::Joints>("ref_" + conf.name);
            ports()->addPort("ref_" + conf.name, *ref_port);
            jnt_ref_ports_[conf.name] = ref_port;

            base::samples::Joints ref;
            ref.resize(conf.joints.size());
            ref.names = conf.joints;
            for(uint i = 0; i < conf.joints.size(); i++)
                ref[i].speed = 0;
            jnt_ref_in_[conf.name] = ref;

            break;
        }
        default:{
            LOG_ERROR("Invalid task type of task %i: %i", i, conf.type);
            return false;
        }
        }//switch

        //Add task weight port
        RTT::InputPort<base::MatrixXd>* weight_port = new RTT::InputPort<base::MatrixXd>("weight_" + conf.name);
        ports()->addPort("weight_" + conf.name, *weight_port);
        weight_ports_[conf.name] = weight_port;
        weight_in_[conf.name] = base::MatrixXd::Identity(no_task_vars, no_task_vars);

        //Add Debug port: Actual Task output from ctrl solution
        RTT::OutputPort<base::VectorXd>* y_task_out_port = new RTT::OutputPort<base::VectorXd>("act_" + conf.name);
        ports()->addPort("act_" + conf.name, *y_task_out_port);
        y_task_out_ports_[conf.name] = y_task_out_port;

        //Add Debug port: Task Jacobian
        RTT::OutputPort<base::MatrixXd>* A_task_out_port = new RTT::OutputPort<base::MatrixXd>("task_mat_" + conf.name);
        ports()->addPort("task_mat_" + conf.name, *A_task_out_port);
        A_task_out_ports_[conf.name] = A_task_out_port;

        wbc_config.push_back(conf);
    }

    LOG_DEBUG("Parsing WBC Config done");
    LOG_DEBUG("WBC Config is: ");
    for(uint i = 0; i < wbc_config.size(); i++){
        LOG_DEBUG("Task %i", i);
        LOG_DEBUG("Name: %s", wbc_config[i].name.c_str());
        LOG_DEBUG("Type: %i", wbc_config[i].type);
        LOG_DEBUG("Priority: %i", wbc_config[i].priority);
        if(wbc_config[i].type == wbc::task_type_cartesian){
            LOG_DEBUG("Root: %s", wbc_config[i].root.c_str());
            LOG_DEBUG("Tip: %s", wbc_config[i].tip.c_str());
        }else{
            LOG_DEBUG("Joints: ");
            for(uint j = 0; j < wbc_config[i].joints.size(); j++)
                LOG_DEBUG("%s", wbc_config[i].joints[j].c_str());
        }
        LOG_DEBUG("\n");
    }

    //
    // Configure wbc lib
    //
    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(urdf_file, tree)){
        LOG_ERROR("Unable to load KDL Tree from urdf file: %s", urdf_file.c_str());
        return false;
    }
    if(!wbc_.configure(tree, wbc_config))
        return false;

    LOG_DEBUG("Configuring WBC Config done");

    //
    // Configure Solver
    //
    solver_.setNormMax(_norm_max.get());
    if(!solver_.configure(wbc_.no_task_vars_pp_, wbc_.no_robot_joints_))
        return false;

    LOG_DEBUG("Configuring Solver Config done");

    joint_status_.resize(wbc_.no_robot_joints_);
    solver_output_.resize(wbc_.no_robot_joints_);
    solver_output_.setZero();
    ctrl_out_.resize(wbc_.no_robot_joints_);
    joint_weights_.resize(wbc_.no_robot_joints_, wbc_.no_robot_joints_);
    joint_weights_.setIdentity();

    return true;
}

bool WbcVelocityTask::startHook()
{
    if (! WbcVelocityTaskBase::startHook())
        return false;
    return true;
}

void WbcVelocityTask::updateHook()
{
    WbcVelocityTaskBase::updateHook();

    base::Time start = base::Time::now();

    //
    // Read inputs
    //
    if(_joint_status.read(joint_status_) == RTT::NoData){
        LOG_DEBUG("No data on joint status port");
        return;
    }

    for(CartPortMap::iterator it = cart_ref_ports_.begin(); it != cart_ref_ports_.end(); it++)
    {
        if(it->second->read(cart_ref_in_[it->first]) == RTT::NewData)
        {
            if(!cart_ref_in_[it->first].hasValidVelocity() ||
               !cart_ref_in_[it->first].hasValidAngularVelocity())
            {
                LOG_ERROR("Reference input of task %s has invalid velocity and/or angular velocity", it->first.c_str());
                throw std::invalid_argument("Invalid Cartesian reference input");
            }
            SubTask* sub_task = wbc_.subTask(it->first);
            sub_task->y_des_.segment(0,3) = cart_ref_in_[it->first].velocity;
            sub_task->y_des_.segment(3,3) = cart_ref_in_[it->first].angular_velocity;
        }
    }

    for(JntPortMap::iterator it = jnt_ref_ports_.begin(); it != jnt_ref_ports_.end(); it++)
    {
        if(it->second->read(jnt_ref_in_[it->first]) == RTT::NewData)
        {
            SubTask* sub_task = wbc_.subTask(it->first);
            if(jnt_ref_in_[it->first].size() != sub_task->no_task_vars_)
            {
                LOG_ERROR("Size for input reference of task %s should be %i but is %i", it->first.c_str(), sub_task->no_task_vars_, jnt_ref_in_[it->first].size());
                throw std::invalid_argument("Invalid joint reference input");
            }

            for(uint i = 0; i < sub_task->no_task_vars_; i++)
            {
                if(!jnt_ref_in_[it->first][i].hasSpeed())
                {
                    LOG_ERROR("Reference input for joint %s of task %s has invalid speed value(s)", jnt_ref_in_[it->first].names[i].c_str(), it->first.c_str());
                    throw std::invalid_argument("Invalid joint reference input");
                }
                sub_task->y_des_(i) = jnt_ref_in_[it->first][i].speed;
            }
        }

    }
    bool has_new_weights = false;
    for(WeightPortMap::iterator it = weight_ports_.begin(); it != weight_ports_.end(); it++)
    {
        if(it->second->read(weight_in_[it->first]) ==  RTT::NewData)
        {
            SubTask* sub_task = wbc_.subTask(it->first);
            if(weight_in_[it->first].size() != sub_task->no_task_vars_)
            {
                LOG_ERROR("Input size for joint weights of task %s should be %i but is %i", it->first.c_str(), sub_task->no_task_vars_, jnt_ref_in_[it->first].size());
                throw std::invalid_argument("Invalid weight input size");
            }
            sub_task->task_weights_ = weight_in_[it->first];
            has_new_weights = true;
        }
    }

    if(_joint_weights.read(joint_weights_) == RTT::NewData)
        solver_.setJointWeights((Eigen::MatrixXd& )joint_weights_);

    //
    // Update equation system
    //
    wbc_.update(joint_status_);

    //
    // Compute control solution
    //
    if(has_new_weights){
        for(uint i = 0; i < wbc_.Wy_.size(); i++)
            solver_.setTaskWeights(wbc_.Wy_[i], i);
    }
    solver_.solve(wbc_.A_, wbc_.y_ref_, (Eigen::VectorXd& )solver_output_);

    //
    // Write output
    //
    ctrl_out_.names = joint_status_.names;
    for(uint i = 0; i < ctrl_out_.size(); i++)
        ctrl_out_[i].speed = solver_output_(i);
    ctrl_out_.time = base::Time::now();
    _ctrl_out.write(ctrl_out_);

    //
    // write Debug Data
    //
    for(AOutPortMap::iterator it = A_task_out_ports_.begin(); it != A_task_out_ports_.end(); it++)
        it->second->write(wbc_.subTask(it->first)->A_);
    for(YOutPortMap::iterator it = y_task_out_ports_.begin(); it != y_task_out_ports_.end(); it++)
        it->second->write(wbc_.subTask(it->first)->A_ * solver_output_);
    for(CartOutPortMap::iterator it = pose_out_ports_.begin(); it != pose_out_ports_.end(); it++){
        base::samples::RigidBodyState rbs;
        SubTask* task = wbc_.subTask(it->first);
        double x,y,w,z;
        task->pose_.M.GetQuaternion(x,y,z,w);
        //cout<<x<<" "<<y<<" "<<z<<" "<<w<<endl<<endl;
        kdl_conversions::KDL2RigidBodyState(task->pose_, rbs);
        rbs.time = base::Time::now();
        rbs.sourceFrame = task->tf_root_->tf_name_;
        rbs.targetFrame = task->tf_tip_->tf_name_;
        it->second->write(rbs);
    }

    _damping.write(solver_.getCurDamping());
    _sample_time.write((base::Time::now() - start).toSeconds());
}

void WbcVelocityTask::cleanupHook()
{
    WbcVelocityTaskBase::cleanupHook();

    for(CartPortMap::iterator it = cart_ref_ports_.begin(); it != cart_ref_ports_.end(); it++)
        delete it->second;

    for(JntPortMap::iterator it = jnt_ref_ports_.begin(); it != jnt_ref_ports_.end(); it++)
        delete it->second;

    for(WeightPortMap::iterator it = weight_ports_.begin(); it != weight_ports_.end(); it++)
        delete it->second;

    for(YOutPortMap::iterator it = y_task_out_ports_.begin(); it != y_task_out_ports_.end(); it++)
        delete it->second;

    for(AOutPortMap::iterator it = A_task_out_ports_.begin(); it != A_task_out_ports_.end(); it++)
        delete it->second;

    cart_ref_ports_.clear();
    cart_ref_in_.clear();
    jnt_ref_ports_.clear();
    jnt_ref_in_.clear();
    weight_ports_.clear();
    weight_in_.clear();
}
