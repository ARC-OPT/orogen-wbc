/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WbcVelocity.hpp"
#include <fstream>
#include <base/logging.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <wbc/SubTaskConfig.hpp>
#include <wbc/RobotModel.hpp>
#include <wbc/HierarchicalWDLSSolver.hpp>

using namespace wbc;

WbcVelocity::WbcVelocity(std::string const& name)
    : WbcVelocityBase(name)
{
    robot_model_ = 0;
}

WbcVelocity::WbcVelocity(std::string const& name, RTT::ExecutionEngine* engine)
    : WbcVelocityBase(name, engine)
{
    robot_model_ = 0;
}

WbcVelocity::~WbcVelocity()
{
}

bool WbcVelocity::configureHook()
{
    if (! WbcVelocityBase::configureHook())
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
    std::vector<srdf::Model::Group> groups = srdf_model.getGroups();
    std::map<std::string, srdf::Model::Group> group_map;
    for(uint i = 0; i < groups.size(); i++)
        group_map[groups[i].name_] = groups[i];

    uint cart_idx = 0, jnt_idx = 0;
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

        switch(conf.type){
        case wbc::task_type_cartesian:{

            if(joint_group.chains_.empty()){
                LOG_ERROR("Task is Cartesian but joint group %s has no chains", group_name.c_str());
                return false;
            }

            if(joint_group.chains_.size() > 1)
                LOG_WARN("No of chains in joint group %s is bigger than 1. Using the first one");

            conf.root = joint_group.chains_[0].first;
            conf.tip = joint_group.chains_[0].second;

            std::stringstream ss;
            ss<<"Cart_"<<conf.priority<<" "<<cart_idx++;
            conf.name = ss.str().c_str();

            //Add cartesian port
            RTT::InputPort<base::samples::RigidBodyState> *ref_port = new RTT::InputPort<base::samples::RigidBodyState>("/Ref_" + conf.name);
            ports()->addPort("/Ref_" + conf.name, *ref_port);
            cart_ref_ports_.push_back(ref_port);
            cart_ref_in_.push_back(base::samples::RigidBodyState());

            //Add weight port
            RTT::InputPort<base::VectorXd>* weight_port = new RTT::InputPort<base::VectorXd>("/Weight_" + conf.name);
            ports()->addPort("/Weight_" + conf.name, *weight_port);
            weight_ports_.push_back(weight_port);
            weight_in_.push_back(base::VectorXd::Ones(6));

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
            std::stringstream ss;
            ss<<"Jnt_"<<conf.priority<<" "<<jnt_idx++;
            conf.name = ss.str().c_str();

            //Add joint port
            RTT::InputPort<base::samples::Joints> *ref_port = new RTT::InputPort<base::samples::Joints>("/Ref_" + conf.name);
            ports()->addPort("/Ref_" + conf.name, *ref_port);
            jnt_ref_ports_.push_back(ref_port);
            base::samples::Joints ref;
            ref.resize(conf.joints.size());
            ref.names = conf.joints;
            jnt_ref_in_.push_back(ref);

            //Add weight port
            RTT::InputPort<base::VectorXd>* weight_port = new RTT::InputPort<base::VectorXd>("/Weight_" + conf.name);
            ports()->addPort("/Weight_" + conf.name, *weight_port);
            weight_ports_.push_back(weight_port);
            weight_in_.push_back(base::VectorXd::Ones(conf.joints.size()));

            break;
        }
        default:{
            LOG_ERROR("Invalid task type of task %i: %i", i, conf.type);
            return false;
        }
        }//switch

        wbc_config_.push_back(conf);
    }

    LOG_DEBUG("Parsing WBC Config done");
    LOG_DEBUG("WBC Config is: ");
    for(uint i = 0; i < wbc_config_.size(); i++){
        LOG_DEBUG("Task %i", i);
        LOG_DEBUG("Name: %s", wbc_config_[i].name.c_str());
        LOG_DEBUG("Type: %i", wbc_config_[i].type);
        LOG_DEBUG("Priority: %s", wbc_config_[i].priority);
        if(wbc_config_[i].type == wbc::task_type_cartesian){
            LOG_DEBUG("Root: %s", wbc_config_[i].root.c_str());
            LOG_DEBUG("Tip: %s", wbc_config_[i].tip.c_str());
        }else{
            LOG_DEBUG("Joints: ");
            for(uint j = 0; j < wbc_config_[i].joints.size(); j++)
                LOG_DEBUG("%s", wbc_config_[i].joints[j].c_str());
        }
        LOG_DEBUG("\n");
    }

    //
    // Create and configure Robot Model
    //
    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(urdf_file, tree)){
        LOG_ERROR("Unable to load KDL Tree from urdf file: %s", urdf_file.c_str());
        return false;
    }
    robot_model_ = new RobotModel(tree);
    if(!robot_model_->configure(wbc_config_)){
        LOG_ERROR("Unable to configure robot model");
        return false;
    }

    //
    // Create and configure solver
    //
    solver_ = new HierarchicalWDLSSolver();
    //solver_->configure();

    // Configure wbc

    return true;
}

bool WbcVelocity::startHook()
{
    if (! WbcVelocityBase::startHook())
        return false;
    return true;
}

void WbcVelocity::updateHook()
{
    WbcVelocityBase::updateHook();
}

void WbcVelocity::cleanupHook()
{
    WbcVelocityBase::cleanupHook();
    wbc_config_.clear();
    delete robot_model_;
}
