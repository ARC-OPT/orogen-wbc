/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/Logging.hpp>

using namespace wbc;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
    //
    // Delete dynamic ports
    //
    for(uint i=0; i<cart_ports.size(); i++)
        delete cart_ports[i];
    for(uint i=0; i<jnt_ports.size(); i++)
        delete jnt_ports[i];
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    std::string srdf = _srdf.get();
    std::string wbc_config = _wbc_config.get();

    //TODO: Parse SRDF

    //TODO: Parse wbc_config, replace dummy data below
    int n_priorities = 5;
    std::vector<int> cart_by_priority;
    std::vector<int> cart_by_priority_chains;
    std::vector<int> joint_by_priority;
    std::vector<int> joint_by_priority_chains;

    //TODO: Verify that srdf and chains defined in wbc_config match
    bool ok = true;

    if(!ok){
        LOG_ERROR("SRDF configuration and wbc_config do not match.");
        return false;
    }

    //
    // Create dynamic ports
    //
    std::stringstream ss;
    for(uint p=0; p<n_priorities; p++){
        for(uint c=0; c<cart_by_priority[p]; c++){
            ss.str("");
            ss<<"cart_"<<p<<"_"<<c;
            RTT::InputPort<base::samples::RigidBodyState>* input_port =
                    new RTT::InputPort<base::samples::RigidBodyState>(ss.str());
            ports()->addPort(ss.str(), *input_port);
            cart_ports.push_back(input_port);
        }
        for(uint c=0; c<joint_by_priority[p]; c++){
            ss.str("");
            ss<<"jnt_"<<p<<"_"<<c;
            RTT::InputPort<base::commands::Joints>* input_port =
                    new RTT::InputPort<base::commands::Joints>(ss.str());
            ports()->addPort(ss.str(), *input_port);
            jnt_ports.push_back(input_port);
        }
    }

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
