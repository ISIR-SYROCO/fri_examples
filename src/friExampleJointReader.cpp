// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExampleJointReader.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/foreach.hpp>

FriExampleJointReader::FriExampleJointReader(std::string const& name) : FriExampleAbstract(name){
    this->addPort("FRIJointState", iport_joint_state);

}

FriExampleJointReader::~FriExampleJointReader(){

}

bool FriExampleJointReader::doStart(){
    return true;
}

void FriExampleJointReader::updateHook(){
    //Read in the iport_joint_state
    unsigned int ndof = 7;
    lwr_fri::FriJointState fri_joint_state_data;
    RTT::FlowStatus jointStateFS = iport_joint_state.read(fri_joint_state_data);
    
    if(jointStateFS == RTT::NewData){
        std::cout << "Measured Joint configuration" << std::endl;
        printData(fri_joint_state_data.msrJntPos);

        std::cout << "Commanded Joint configuration before FRI" << std::endl;
        printData(fri_joint_state_data.cmdJntPos);

        std::cout << "Commanded Joint configuration FRI offset" << std::endl;
        printData(fri_joint_state_data.cmdJntPosFriOffset);

        std::cout << "Measured Joint Torque" << std::endl;
        printData(fri_joint_state_data.msrJntTrq);

        std::cout << "Estimated external joint torque" << std::endl;
        printData(fri_joint_state_data.estExtJntTrq);
        
        std::cout << "Gravity compensation" << std::endl;
        printData(fri_joint_state_data.gravity);
    }
}

void FriExampleJointReader::printData(boost::array<float, 7> &vec){
    BOOST_FOREACH(float f, vec){
        std::cout << f << " ";
    }
    std::cout << std::endl;
}

void FriExampleJointReader::stopHook(){

}

void FriExampleJointReader::cleanupHook(){

}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(component)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(FriExampleJointReader)
