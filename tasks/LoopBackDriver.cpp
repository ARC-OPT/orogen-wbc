/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "LoopBackDriver.hpp"

using namespace wbc;

LoopBackDriver::LoopBackDriver(std::string const& name)
    : LoopBackDriverBase(name){
}

LoopBackDriver::~LoopBackDriver(){
}

double whiteNoise(const double std_dev){
    double rand_no = ( rand() / ( (double)RAND_MAX ) );
    while( rand_no == 0 )
        rand_no = ( rand() / ( (double)RAND_MAX ) );

    double tmp = cos( ( 2.0 * (double)M_PI ) * rand() / ( (double)RAND_MAX ) );
    return std_dev * sqrt( -2.0 * log( rand_no ) ) * tmp;
}

bool LoopBackDriver::configureHook(){
    if (! LoopBackDriverBase::configureHook())
        return false;

    joint_state = _initial_joint_state.get();
    noise_std_dev = _noise_std_dev.get();

    return true;
}

bool LoopBackDriver::startHook(){
    if (! LoopBackDriverBase::startHook())
        return false;
    return true;
}

void LoopBackDriver::updateHook(){
    LoopBackDriverBase::updateHook();

    if(_command.readNewest(command) == RTT::NewData){
        for(auto n : joint_state.names)
            joint_state[n] = command[n];
    }

    for(auto n: joint_state.names){
        joint_state[n].position += whiteNoise(noise_std_dev);
        joint_state[n].speed += whiteNoise(noise_std_dev);
        joint_state[n].acceleration += whiteNoise(noise_std_dev);
    }
    joint_state.time = base::Time::now();
    _joint_state.write(joint_state);
}

void LoopBackDriver::errorHook(){
    LoopBackDriverBase::errorHook();
}

void LoopBackDriver::stopHook(){
    LoopBackDriverBase::stopHook();
}

void LoopBackDriver::cleanupHook(){
    LoopBackDriverBase::cleanupHook();
}
