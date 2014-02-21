/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include <wbc/SubTask.hpp>
#include "WbcVelocityTask.hpp"
#include <base/logging.h>
#include <wbc/TaskFrame.hpp>
#include <urdf_parser/urdf_parser.h>
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
    std::vector<wbc::SubTaskConfig> wbc_config = _wbc_config.get();

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

    for(uint i = 0; i < wbc_config.size(); i++){

        SubTaskConfig conf = wbc_config[i];

        std::string port_namespace;
        uint no_task_vars;

        switch(conf.type){
        case wbc::task_type_cartesian:{

            no_task_vars = 6;

            //Port naming
            std::stringstream ss;
            ss<<"p"<<conf.priority<<"_cart_"<<conf.name;
            port_namespace = ss.str();

            //Add cartesian reference port
            RTT::InputPort<base::samples::RigidBodyState> *ref_port = new RTT::InputPort<base::samples::RigidBodyState>("ref_" + port_namespace);
            ports()->addPort("ref_" + port_namespace, *ref_port);
            cart_ref_ports_[conf.name] = ref_port;

            //Add Debug port: Task Pose
            RTT::OutputPort<base::samples::RigidBodyState>* pose_out_port = new RTT::OutputPort<base::samples::RigidBodyState>("pose_" + port_namespace);
            ports()->addPort("pose_" + port_namespace, *pose_out_port);
            pose_out_ports_[conf.name] = pose_out_port;

            kdl_conversions::KDL2RigidBodyState(KDL::Twist::Zero(), cart_ref_in_[conf.name]);
            break;
        }
        case wbc::task_type_joint:{

            no_task_vars = conf.joint_names.size();

            //Port naming
            std::stringstream ss;
            ss<<"p"<<conf.priority<<"_jnt_"<<conf.name;
            port_namespace = ss.str();

            //Add joint reference port
            RTT::InputPort<base::samples::Joints> *ref_port = new RTT::InputPort<base::samples::Joints>("ref_" + port_namespace);
            ports()->addPort("ref_" + port_namespace, *ref_port);
            jnt_ref_ports_[conf.name] = ref_port;

            base::samples::Joints ref;
            ref.resize(conf.joint_names.size());
            ref.names = conf.joint_names;
            for(uint i = 0; i < conf.joint_names.size(); i++)
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
        RTT::InputPort<base::VectorXd>* weight_port = new RTT::InputPort<base::VectorXd>("weight_" + port_namespace);
        ports()->addPort("weight_" + port_namespace, *weight_port);
        weight_ports_[conf.name] = weight_port;
        weight_in_[conf.name] = base::VectorXd::Ones(no_task_vars);

        //Add Debug port: Actual Task output from ctrl solution
        RTT::OutputPort<base::VectorXd>* y_task_out_port = new RTT::OutputPort<base::VectorXd>("act_" + port_namespace);
        ports()->addPort("act_" + port_namespace, *y_task_out_port);
        y_task_out_ports_[conf.name] = y_task_out_port;

        //Add Debug port: Task Jacobian
        RTT::OutputPort<base::MatrixXd>* A_task_out_port = new RTT::OutputPort<base::MatrixXd>("task_mat_" + port_namespace);
        ports()->addPort("task_mat_" + port_namespace, *A_task_out_port);
        A_task_out_ports_[conf.name] = A_task_out_port;

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
            for(uint j = 0; j < wbc_config[i].joint_names.size(); j++)
                LOG_DEBUG("%s", wbc_config[i].joint_names[j].c_str());
        }
        LOG_DEBUG("\n");
    }

    //
    // Configure wbc lib
    //
    KDL::Tree full_tree, tree;
    if(!kdl_parser::treeFromFile(urdf_file, full_tree)){
        LOG_ERROR("Unable to load KDL Tree from urdf file: %s", urdf_file.c_str());
        return false;
    }
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

    if(!wbc_.configure(tree, wbc_config, _joint_names.get()))
        return false;

    //In wbc, tasks are active by default (all task weights are 1). Deactivate them if desired by user
    if(!_tasks_active.get())
    {
        for(SubTaskMap::iterator it = wbc_.sub_task_map_.begin(); it != wbc_.sub_task_map_.end(); it++)
            it->second->task_weights_.setZero();
    }

    LOG_DEBUG("Configuring WBC Config done");

    //
    // Configure Solver
    //
    solver_.setNormMax(_norm_max.get());
    if(!solver_.configure(wbc_.no_task_vars_pp_, wbc_.no_robot_joints_))
        return false;

    base::VectorXd joint_weight_vect = _initial_joint_weights.get();
    if(joint_weight_vect.size() != 0)
    {
        base::MatrixXd joint_weights(joint_weight_vect.size(), joint_weight_vect.size());
        joint_weights.setIdentity();
        joint_weights.diagonal() = joint_weight_vect;
        solver_.setJointWeights(joint_weights);
    }

    LOG_DEBUG("Configuring Solver Config done");

    solver_output_.resize(wbc_.no_robot_joints_);
    solver_output_.setZero();
    joint_weights_ = base::VectorXd::Ones(wbc_.no_robot_joints_);
    joint_weight_mat_.resize(wbc_.no_robot_joints_, wbc_.no_robot_joints_);
    joint_weight_mat_.setIdentity();

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
    if(_joint_state.read(joint_status_) == RTT::NoData){
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
            sub_task->task_weights_.diagonal() = weight_in_[it->first];
            has_new_weights = true;
        }
    }

    if(_joint_weights.read(joint_weights_) == RTT::NewData){
        joint_weight_mat_.diagonal() = joint_weights_;
        solver_.setJointWeights((Eigen::MatrixXd& )joint_weight_mat_);
    }

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
    if(ctrl_out_.empty()){
        ctrl_out_.resize(joint_status_.size());
        ctrl_out_.names = joint_status_.names;
    }
    for(uint i = 0; i < ctrl_out_.size(); i++){
        uint idx = wbc_.joint_index_map_[ctrl_out_.names[i]];
        ctrl_out_[i].speed = solver_output_(idx);
    }
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
