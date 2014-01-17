// Copyright 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExampleAbstract.hpp"

FriExampleAbstract::FriExampleAbstract(std::string const& name) : RTT::TaskContext(name){
    this->addPort("fromFRI", port_fri_to_krl);
    this->addPort("toFRI", port_fri_frm_krl);

    this->addOperation("getControlStrategy", &FriExampleAbstract::getControlStrategy, this, RTT::OwnThread);
    this->addOperation("getFRIMode", &FriExampleAbstract::getFRIMode, this, RTT::OwnThread);

    this->addOperation("setControlStrategy", &FriExampleAbstract::setControlStrategy, this, RTT::OwnThread);
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
    //Set control strategy to joint position
    fri_to_krl.intData[1]=10;

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

void FriExampleAbstract::setControlStrategy(int mode){
    if(mode != 10 && mode != 20 && mode != 30){
        std::cout << "Please set a valid control mode: " << std::endl;
        std::cout << "10: Joint position" << std::endl;
        std::cout << "20: Cartesian stiffness" << std::endl;
        std::cout << "30: Joint stiffness" << std::endl;
        return;
    }
    else{
        fri_to_krl.intData[1] = mode;
        controlMode = mode;
        port_fri_to_krl.write(fri_to_krl);
    }
}

bool FriExampleAbstract::requiresControlMode(int modeRequired){
    if (controlMode == modeRequired){
        return true;
    }
    else{
        std::cout << "Cannot proceed, current control mode is " << controlMode
            << " required control mode is " << modeRequired << std::endl;
        return false;
    }
}

void FriExampleAbstract::getFRIMode(){
    RTT::FlowStatus fri_frm_krl_fs = port_fri_frm_krl.read(fri_frm_krl);
    if(fri_frm_krl_fs == RTT::NewData){
        if(fri_frm_krl.intData[1] == 1){
            std::cout << "FRI in Command Mode" << std::endl;
        }
        else if(fri_frm_krl.intData[1] == 2){
            std::cout << "FRI in Monitor Mode" << std::endl;
        }
    }
}

void FriExampleAbstract::getControlStrategy(){
    RTT::FlowStatus fri_frm_krl_fs = port_fri_frm_krl.read(fri_frm_krl);
    if(fri_frm_krl_fs == RTT::NewData){
        if(fri_frm_krl.intData[2] == 10){
            std::cout << "Joint position control" << std::endl;
        }
        else if(fri_frm_krl.intData[2] == 20){
            std::cout << "Cartesian impedance control" << std::endl;
        }
        else if(fri_frm_krl.intData[2] == 30){
            std::cout << "Joint impedance control" << std::endl;
        }
    }
}
