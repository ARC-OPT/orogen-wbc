/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/Logging.hpp>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <base/Eigen.hpp>
#include <fstream>

using namespace wbc;

Task::Task(std::string const& name)
    : TaskBase(name),
      wbc_(0){
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine),
      wbc_(0){
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    std::string urdf_file = _urdf.get();
    std::string srdf_file = _srdf.get();
    std::string wbc_config_file = _wbc_config.get();

    //
    // Load urdf model: Kinematics and dynamics of the robot
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
    if(!srdf_model_.initFile(*urdf_model.get(), srdf_file)){
        LOG_ERROR("Error opening srdf file %s", srdf_file.c_str());
        return false;
    }

    LOG_DEBUG("Parsing SRDF file done");

    //
    // Parse Wbc Config: Defines all sub tasks in the whole body control problem
    //
    std::ifstream wbc_conf_stream(wbc_config_file.c_str());
    if(!wbc_conf_stream.is_open()){
        LOG_ERROR("Unable to open wbc config file %s",wbc_config_file.c_str());
        return false;
    }
    YAML::Parser parser(wbc_conf_stream);
    YAML::Node doc;
    if(!parser.GetNextDocument(doc))
        return false;

    wbc_config_.clear();
    wbc_config_.resize(doc.size());
    for(uint prio = 0; prio < doc.size(); prio++){

        const YAML::Node& prio_node = doc[prio];
        for(uint i = 0; i < prio_node.size(); i++)
            wbc_config_[prio].push_back(parseSubTaskConfig(prio_node[i]));
    }

    LOG_DEBUG("Parsing WBC configuration done");

    LOG_DEBUG("");
    LOG_DEBUG("------- WBC Configuration: ------- \n");
    for(uint prio = 0; prio < wbc_config_.size(); prio++){
        LOG_DEBUG("Priority: %i\n", prio);
        for(uint i = 0; i < wbc_config_[prio].size(); i++){
            SubTaskConfig c = wbc_config_[prio][i];
            LOG_DEBUG("Sub Task no: %i", i);
            std::string type;
            c.type == cartesian ? type = "cartesian" : type = "joint";
            LOG_DEBUG("Type: %s", type.c_str());
            LOG_DEBUG("Root: %s", c.root.c_str());
            LOG_DEBUG("Tip: %s", c.tip.c_str());
            LOG_DEBUG("Joints: ");
            for(uint j = 0; j < c.joints.size(); j++)
                LOG_DEBUG("%s", c.joints[j].c_str());
            LOG_DEBUG("");
        }
        LOG_DEBUG("");
    }
    LOG_DEBUG("----------------------------------------\n");

    //
    // Parse URDF to KDL
    //
    KDL::Tree robot_tree;
    if(!kdl_parser::treeFromUrdfModel(*urdf_model.get(), robot_tree)){
        LOG_ERROR("Unable to init KDL tree from file %s", urdf_file.c_str());
        return false;
    }

    LOG_DEBUG("Creating KDL model done");

    //
    // Create and configure wbc lib
    //
    mode wbc_mode = _wbc_mode.get();
    wbc_ = new Wbc(robot_tree, wbc_mode);
    if(!wbc_->configure(wbc_config_))
        return false;

    //
    // Create dynamic ports and data structures
    //
    std::stringstream ss;
    reference_.resize(wbc_config_.size());
    task_weights_.resize(wbc_config_.size());

    for(uint prio = 0; prio < wbc_config_.size(); prio++){

        ss.str("");
        ss<<"A_"<<prio;
        RTT::OutputPort<base::MatrixXd>* A_port = new RTT::OutputPort<base::MatrixXd>(ss.str());
        ports()->addPort(ss.str(), *A_port);
        A_ports_.push_back(A_port);
        LOG_DEBUG("Created debug port %s", ss.str().c_str());

        ss.str("");
        ss<<"Wy_"<<prio;
        RTT::OutputPort<base::VectorXd>* Wy_port = new RTT::OutputPort<base::VectorXd>(ss.str());
        ports()->addPort(ss.str(), *Wy_port);
        Wy_ports_.push_back(Wy_port);
        LOG_DEBUG("Created debug port %s", ss.str().c_str());

        ss.str("");
        ss<<"y_ref_"<<prio;
        RTT::OutputPort<base::VectorXd>* y_ref_port = new RTT::OutputPort<base::VectorXd>(ss.str());
        ports()->addPort(ss.str(), *y_ref_port);
        y_ref_ports_.push_back(y_ref_port);
        LOG_DEBUG("Created debug port %s", ss.str().c_str());

        ss.str("");
        ss<<"y_"<<prio;
        RTT::OutputPort<base::VectorXd>* y_port = new RTT::OutputPort<base::VectorXd>(ss.str());
        ports()->addPort(ss.str(), *y_port);
        y_ports_.push_back(y_port);
        LOG_DEBUG("Created debug port %s", ss.str().c_str());

        for(uint c = 0; c < wbc_config_[prio].size(); c++){

            ss.str("");

            SubTaskConfig conf = wbc_config_[prio][c];

            ss<<"command_"<<prio<<"_"<<c;
            RTT::InputPort<base::VectorXd>* ref_port = new RTT::InputPort<base::VectorXd>(ss.str());
            ports()->addPort(ss.str(), *ref_port);
            ref_ports_.push_back(ref_port);
            LOG_DEBUG("Created command port %s", ss.str().c_str());

            ss.str("");
            ss<<"weight_"<<prio<<"_"<<c;
            RTT::InputPort<base::VectorXd>* weight_port = new RTT::InputPort<base::VectorXd>(ss.str());
            ports()->addPort(ss.str(), *weight_port);
            weight_ports_.push_back(weight_port);
            LOG_DEBUG("Created weight port %s", ss.str().c_str());

            uint nt;
            conf.type == cartesian ? nt = 6 : nt =  conf.joints.size();
            reference_[prio].push_back(base::VectorXd::Zero(nt));
            task_weights_[prio].push_back(base::VectorXd::Ones(nt));
        }
    }

    status_.resize(wbc_->robot_->no_of_joints_);
    solver_output_.resize(wbc_->robot_->no_of_joints_);
    joint_weights_.resize(wbc_->robot_->no_of_joints_);
    joint_weights_.setConstant(1);

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    if(!_joint_status.connected()){
        LOG_ERROR("Joint Status port is not connected");
        return false;
    }

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    if(_joint_status.read(status_) == RTT::NoData){
        LOG_DEBUG("No data on joint status port");
        return;
    }

    _joint_weights.read(joint_weights_);

    //Read all task ports
    uint port_idx = 0;
    for(uint p = 0; p < wbc_config_.size(); p++){
        for(uint i = 0; i < wbc_config_[p].size(); i++){
            ref_ports_[port_idx]->read(reference_[p][i]);
            weight_ports_[port_idx]->read(task_weights_[p][i]);
            port_idx++;
        }
    }

    //Compute control solution
    wbc_->solve(reference_,
                task_weights_,
                joint_weights_,
                status_,
                solver_output_);

    _command.write(solver_output_);

    //Write debug ports
    for(uint prio = 0; prio < wbc_config_.size(); prio++){
        A_ports_[prio]->write(wbc_->A_[prio]);
        Wy_ports_[prio]->write(wbc_->Wy_[prio]);
        y_ref_ports_[prio]->write(wbc_->y_ref_[prio]);
        y_ports_[prio]->write(wbc_->y_[prio]);
    }
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    // Delete dynamic ports
    for(uint i=0; i<ref_ports_.size(); i++)
        delete ref_ports_[i];
    for(uint i=0; i<weight_ports_.size(); i++)
        delete weight_ports_[i];

    wbc_config_.clear();
    delete wbc_;
    wbc_ = 0;
}

SubTaskConfig Task::parseSubTaskConfig(const YAML::Node& node){

    //put all srdf groups in a map
    std::vector<srdf::Model::Group> groups = srdf_model_.getGroups();
    std::map<std::string, srdf::Model::Group> group_map;
    for(uint i = 0; i < groups.size(); i++)
        group_map[groups[i].name_] = groups[i];

    std::string group_name;
    if(!node.FindValue("group")){
        std::stringstream ss;
        ss<<"Error when parsing wbc config: Key 'group' not found"<<std::endl;
        throw std::invalid_argument(ss.str());
    }
    node["group"] >> group_name;

    //Check if group name is available in srdf file
    if(group_map.count(group_name) == 0){
        LOG_ERROR("No such joint group in srdf: %s", group_name.c_str());
        throw std::invalid_argument("No such joint group");
    }

    srdf::Model::Group joint_group = group_map[group_name];

    std::string type;
    if(!node.FindValue("type")){
        std::stringstream ss;
        ss<<"Error when parsing wbc config: Key 'type' not found"<<std::endl;
        throw std::invalid_argument(ss.str());
    }
    node["type"] >> type; //cartesian or joint space task?

    LOG_INFO("%s", type.c_str());

    SubTaskConfig config;
    if(type.compare("cartesian") == 0)
        config.type = cartesian;
    else if(type.compare("joint") == 0)
        config.type = joint;
    else{
        std::stringstream ss;
        ss<<"Invalid task type: "<<type<<std::endl;
        throw std::invalid_argument(ss.str());
    }

    switch(config.type){
    case cartesian:{

        //Only accept chains from srdf as input for Cartesian tasks
        if(joint_group.chains_.empty()){
            std::stringstream ss;
            ss<<"Task is Cartesian but joint group "<<group_name.c_str()<<" has no chains"<<std::endl;
            throw std::invalid_argument(ss.str());
        }
        else{
            if(joint_group.chains_.size() > 1)
                LOG_WARN("No of chains in joint group %s is bigger than 1. Using the first one");

            config.root = joint_group.chains_[0].first;
            config.tip = joint_group.chains_[0].second;
        }
        break;
    }
    case joint:{
        //Only accept joints from srdf as input for joint space task
        if(joint_group.joints_.empty()){
            std::stringstream ss;
            ss<<"Task is in joint space but joint group "<<group_name.c_str()<<" has no joints"<<std::endl;
            throw std::invalid_argument(ss.str());
        }
        else
            config.joints = joint_group.joints_;
        break;
    }
    default:{
        throw("Invalid Task type");
        break;
    }
    }
    return config;
}


