// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExample.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/foreach.hpp>
#include <kdl/frames_io.hpp>

FriExample::FriExample(std::string const& name) : FriExampleAbstract(name){
    this->addOperation("getFRIJointState", &FriExample::getFRIJointState, this, RTT::OwnThread);
    this->addOperation("getCartesianPosition", &FriExample::getCartesianPosition, this, RTT::OwnThread);
    this->addOperation("getCartesianFrame", &FriExample::getCartesianFrame, this, RTT::OwnThread);
    this->addOperation("getRobotState", &FriExample::getRobotState, this, RTT::OwnThread);
    this->addOperation("getJointState", &FriExample::getJointState, this, RTT::OwnThread);
    this->addOperation("getCartesianWrench", &FriExample::getCartesianWrench, this, RTT::OwnThread);
    this->addOperation("getJacobian", &FriExample::getJacobian, this, RTT::OwnThread);
}

FriExample::~FriExample(){
}

void FriExample::updateHook(){
}

void FriExample::getFRIJointState(){
    lwr_fri::FriJointState fri_joint_state_data;
    RTT::FlowStatus fri_jointStateFS = iport_fri_joint_state.read(fri_joint_state_data);

    if(fri_jointStateFS == RTT::NewData){
        std::cout << "Measured Joint configuration" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.msrJntPos){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Commanded Joint configuration before FRI" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.cmdJntPos){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Commanded Joint configuration FRI offset" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.cmdJntPosFriOffset){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Measured Joint torque actuator" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.msrJntTrq){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Estimated External Joint torque sensor" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.estExtJntTrq){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Gravity compensation" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.gravity){
            std::cout << f << " ";
        }
        std::cout << std::endl;
    }
}

void FriExample::getCartesianPosition(){
}

void FriExample::getCartesianFrame(){
    KDL::Frame cart_frame_data;
    RTT::FlowStatus cartFrameFS = iport_cart_frame.read(cart_frame_data);
    if(cartFrameFS == RTT::NewData){
        std::cout << "Cartesian Frame end effector" << std::endl;
        std::cout << cart_frame_data << std::endl;
    }

}

void FriExample::getRobotState(){

}

void FriExample::getJointState(){

}

void FriExample::getCartesianWrench(){

}

void FriExample::getJacobian(){

}

ORO_CREATE_COMPONENT(FriExample)
