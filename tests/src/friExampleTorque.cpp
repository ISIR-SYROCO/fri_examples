// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExampleTorque.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/foreach.hpp>
#include <math.h>

FriExampleKinematic::FriExampleKinematic(std::string const& name) : FriExampleAbstract(name){
    this->addPort("FRIJointState_i", iport_fri_joint_state);
    this->addPort("JointState_i", iport_joint_state);

    this->addPort("JointPositions_o", oport_joint_position);
    this->addPort("JointVelocities_o", oport_joint_velocities);
    this->addPort("JointTorques_o", oport_joint_efforts);
    this->addPort("JointImpedance_o", oport_joint_impedance);

    this->addOperation("setJointImpedance", &FriExampleKinematic::setJointImpedance, this, RTT::OwnThread);

}

FriExampleKinematic::~FriExampleKinematic(){
}

bool FriExampleKinematic::doStart(){
}

void FriExampleKinematic::updateHook(){
}

void FriExampleKinematic::setJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping){
    if(stiffness.size() != 7 || damping.size() != 7){
        std::cout << "Wrong vector size, should be 7,7" << std::endl;
        return;
    }
    else{
        lwr_fri::FriJointImpedance joint_impedance_command;
        for(unsigned int i = 0; i < 7; i++){
            joint_impedance_command.stiffness[i] = stiffness[i];
            joint_impedance_command.damping[i] = damping[i];
        }

        oport_joint_impedance.write(joint_impedance_command);
    }
}

ORO_CREATE_COMPONENT(FriExampleKinematic)

