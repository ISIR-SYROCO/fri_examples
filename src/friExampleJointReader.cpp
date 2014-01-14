// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExampleJointReader.hpp"
#include <rtt/Component.hpp>
#include <iostream>
using namespace RTT;
FriExampleJointReader::FriExampleJointReader(std::string const& name) : FriExampleAbstract(name){

}

FriExampleJointReader::~FriExampleJointReader(){

}

bool FriExampleJointReader::doStart(){
    return true;
}

void FriExampleJointReader::updateHook(){

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
