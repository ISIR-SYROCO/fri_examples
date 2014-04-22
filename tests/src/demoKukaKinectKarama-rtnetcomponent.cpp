// Filename:  demoKukaKinectKarama-rtnetcomponent.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) from Karama Sriti
// Description:  

#include "demoKukaKinectKarama-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>

DemoKukaKinectKaramaRTNET::DemoKukaKinectKaramaRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
    this->addPort("distance", port_distance);
    this->addPort("dummy_distance_out", port_dummy_double);
    this->addOperation("setNumObs", &DemoKukaKinectKaramaRTNET::setNumObs, this, RTT::OwnThread);

    distance = 0.0;
    direction = 1;
    setNumObs(20);

    m_joint_vel_command.resize(LWRDOF);
    std::fill(m_joint_vel_command.begin(), m_joint_vel_command.end(), 0.0);
}

void DemoKukaKinectKaramaRTNET::updateHook(){
   fri_frm_krl = m_fromFRI.get(); 
   if(fri_frm_krl.intData[0] == 1){ //command mode

       RTT::FlowStatus distanceFS = port_distance.read(distance);

       std::vector<double> JState(LWRDOF);

       RTT::FlowStatus joint_state_fs =iport_msr_joint_pos.read(JState);

       if(joint_state_fs == RTT::NewData){
           if(JState[2] > 1.0471975512 && direction == 1){
               direction = -1;
           }
           else if( JState[2] < -1.0471975512 && direction == -1){
               direction = 1;
           }

           double alpha = std::min( std::max(distance, 0.15), 1.0 ) - 0.15;
           m_joint_vel_command[2] = mean.getMean( 0.4 * alpha * direction );

           oport_joint_velocities.write(m_joint_vel_command);

       }
   }

}

void DemoKukaKinectKaramaRTNET::setNumObs(unsigned int numObs){
    mean.setNumObs(numObs);
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Kuka_send_joint_positions)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(DemoKukaKinectKaramaRTNET)
