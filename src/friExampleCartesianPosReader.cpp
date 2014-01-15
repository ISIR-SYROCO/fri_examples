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
}

ORO_CREATE_COMPONENT(FriExampleCartesianPosReader)
