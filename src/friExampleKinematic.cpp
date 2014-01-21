// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExampleKinematic.hpp"
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

    this->addOperation("setDesiredQ", &FriExampleKinematic::setDesiredQ, this, RTT::OwnThread);
    this->addOperation("setJointImpedance", &FriExampleKinematic::setJointImpedance, this, RTT::OwnThread);

    this->addOperation("setLambda", &FriExampleKinematic::setLambda, this, RTT::OwnThread);

    qdes.resize(7);
    lambda = 0.05;
}

FriExampleKinematic::~FriExampleKinematic(){
}

bool FriExampleKinematic::doStart(){
    sensor_msgs::JointState joint_state_data;
    RTT::FlowStatus joint_state_fs = iport_joint_state.read(joint_state_data);
    if(joint_state_fs == RTT::NewData){
        std::vector<double>::iterator it = qdes.begin();
        BOOST_FOREACH(double d, joint_state_data.position){
            *it = d;
            ++it;
        }
    }
    else{
        std::cout << "Cannot read robot position, fail to start" << std::endl;
        return false;
    }
}

void FriExampleKinematic::updateHook(){
    motion_control_msgs::JointVelocities command;
    command.velocities.assign(7, 0.0);

    std::vector<double> qerror(7, 0.0);
    sensor_msgs::JointState joint_state_data;
    RTT::FlowStatus joint_state_fs = iport_joint_state.read(joint_state_data);

    if(joint_state_fs == RTT::NewData){
        for(int i = 0; i < 7; i++){
            qerror[i] = qdes[i] - joint_state_data.position[i];
        }
        double normQerror = 0;
        BOOST_FOREACH(double d, qerror){
            normQerror += d*d;
            normQerror = sqrt(normQerror);
        }
        if(normQerror < 0.1){
            oport_joint_velocities.write(command); 
            return;
        }
        else{
            for(int i = 0; i < 7; i++){
                command.velocities[i] = lambda * qerror[i];
            }
            oport_joint_velocities.write(command);
        }
    }
}

void FriExampleKinematic::setDesiredQ(std::vector<double> &qdes_){
    std::vector<double>::iterator it = qdes.begin();
    BOOST_FOREACH(double d, qdes_){
        *it = d;
        ++it;
    }
}

void FriExampleKinematic::setLambda(double l){
    lambda = l;
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

