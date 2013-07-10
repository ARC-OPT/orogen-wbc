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

    int priorities = _priorities.get();
    std::vector<int> cart_input = _cart_input.get();
    std::vector<int> jnt_input = _jnt_input.get();

    if(priorities != cart_input.size() || priorities != jnt_input.size()){
        LOG_ERROR("Configuration has an error. priorities != cart_input.size() || priorities != jnt_input.size()");
        return false;
    }

    //
    // Create dynamic ports
    //
    std::stringstream ss;
    for(uint p=0; p<priorities; p++){
        for(uint c=0; c<cart_input[p]; c++){
            ss.str("");
            ss<<"cart_"<<p<<"_"<<c;
            RTT::InputPort<base::samples::RigidBodyState>* input_port =
                    new RTT::InputPort<base::samples::RigidBodyState>(ss.str());
            ports()->addPort(ss.str(), *input_port);
            cart_ports.push_back(input_port);
        }
        for(uint c=0; c<cart_input[p]; c++){
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
