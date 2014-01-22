// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExampleTorque.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/foreach.hpp>
#include <math.h>

FriExampleTorque::FriExampleTorque(std::string const& name) : FriExampleAbstract(name){
    this->addPort("FRIJointState_i", iport_fri_joint_state);
    this->addPort("JointState_i", iport_joint_state);

    this->addPort("JointPositions_o", oport_joint_position);
    this->addPort("JointVelocities_o", oport_joint_velocities);
    this->addPort("JointTorques_o", oport_joint_efforts);
    this->addPort("JointImpedance_o", oport_joint_impedance);

    this->addOperation("setJointImpedance", &FriExampleTorque::setJointImpedance, this, RTT::OwnThread);

}

FriExampleTorque::~FriExampleTorque(){
}

bool FriExampleTorque::doStart(){
    //setting stiffness
    std::cout << "Setting the stiffness and damping" << std::endl;
    std::vector<double> stiff(LWRDOF, 100.0);
    std::vector<double> damp(LWRDOF, 0.7);
    setJointImpedance(stiff, damp);
    return true;
}

void FriExampleTorque::updateHook(){
    if(requiresControlMode(30)){
        //Get gravity terms
        lwr_fri::FriJointState fri_joint_state_data;
        RTT::FlowStatus fri_jointStateFS = iport_fri_joint_state.read(fri_joint_state_data);

        motion_control_msgs::JointEfforts joint_eff_command;

        if(fri_jointStateFS == RTT::NewData){
            for(int i = 0; i < LWRDOF; i++){
                joint_eff_command.efforts[i] = fri_joint_state_data.gravity[i]; 
            }
        }

    }
}

void FriExampleTorque::setJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping){
    if(stiffness.size() != LWRDOF || damping.size() != LWRDOF){
        std::cout << "Wrong vector size, should be " <<  LWRDOF << ", " << LWRDOF << std::endl;
        return;
    }
    else{
        lwr_fri::FriJointImpedance joint_impedance_command;
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_impedance_command.stiffness[i] = stiffness[i];
            joint_impedance_command.damping[i] = damping[i];
        }

        oport_joint_impedance.write(joint_impedance_command);
    }
}

ORO_CREATE_COMPONENT(FriExampleTorque)

