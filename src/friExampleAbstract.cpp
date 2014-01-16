// Copyright 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExampleAbstract.hpp"

FriExampleAbstract::FriExampleAbstract(std::string const& name) : RTT::TaskContext(name){
    this->addPort("fromFRI", port_fri_to_krl);
    this->addPort("toFRI", port_fri_frm_krl);
}

FriExampleAbstract::~FriExampleAbstract(){
}

bool FriExampleAbstract::configureHook(){
    //initialize the arrays that will be send to KRL
    for(int i=0; i<16; ++i){
        fri_to_krl.intData[i]=0;
        fri_to_krl.realData[i]=0.0;
    }

    //We chose to put 1 on the $FRI_FRM_INT[1] to trigger the fri_start()
    //In KRL, index starts at 1
    fri_to_krl.intData[0]=1;

    return true;
}

bool FriExampleAbstract::startHook(){
    //Send arrays to KRC
    port_fri_to_krl.write(fri_to_krl);
    return doStart();
}

bool FriExampleAbstract::doStart(){
    return true;
}

void FriExampleAbstract::stopHook(){
    doStop();
    //Put 2 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=2;
    port_fri_to_krl.write(fri_to_krl);
}

void FriExampleAbstract::doStop(){}

void FriExampleAbstract::cleanupHook(){}

