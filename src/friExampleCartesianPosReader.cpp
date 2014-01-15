// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExampleCartesianPosReader.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/foreach.hpp>

FriExampleCartesianPosReader::FriExampleCartesianPosReader(std::string const& name) : FriExampleAbstract(name){
    this->addPort("CartesianPosition", iport_cart_pos);

}

FriExampleCartesianPosReader::~FriExampleCartesianPosReader(){
}

void FriExampleCartesianPosReader::updateHook(){
    geometry_msgs::Pose cartPosData;
    RTT::FlowStatus cartPosFS = iport_cart_pos.read(cartPosData);

    if(cartPosFS == RTT::NewData){
        std::cout << "Position end effector [x, y, z]:" << std::endl;
        std::cout << "[" << cartPosData.position.x
                  << ", " << cartPosData.position.y 
                  << ", " << cartPosData.position.z
                  << "]" << std::endl;
        std::cout << "Orientation end effector [x, y, z, w]:" << std::endl;
        std::cout << "[" << cartPosData.orientation.x
                  << ", " << cartPosData.orientation.y
                  << ", " << cartPosData.orientation.z
                  << ", " << cartPosData.orientation.w
                  << "]" << std::endl;
    }
}

ORO_CREATE_COMPONENT(FriExampleCartesianPosReader)
