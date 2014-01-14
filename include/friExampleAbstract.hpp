// Copyright 2014 ISIR-CNRS
// Author: Sovannara Hak

#ifndef OROCOS_FRI_EXAMPLE_ABSTRACT_HPP
#define OROCOS_FRI_EXAMPLE_ABSTRACT_HPP

#include <rtt/RTT.hpp>
#include <kuka_lwr_fri/friComm.h>

class FriExampleAbstract : public RTT::TaskContext{
  public:
    /**
     * Shared arrays from the remote pc to the KRC
     */
    tFriKrlData fri_to_krl;

    /**
     * Shared arrays from the KRC to the remote pc
     */
    tFriKrlData fri_frm_krl;

    /**
     * Output port to send shared arrays to the KRC
     */
    RTT::OutputPort<tFriKrlData> port_fri_to_krl;

    /**
     * Input port to read shared arrays from the KRC
     */
    RTT::InputPort<tFriKrlData> port_fri_frm_krl;

    FriExampleAbstract(std::string const& name) : RTT::TaskContext(name){
        this->addPort("fromFRI", port_fri_to_krl);
        this->addPort("toFRI", port_fri_frm_krl);
    }

    ~FriExampleAbstract(){
    }

    /** @brief Orocos Configure Hook
     * Initialization of the shared array between
     * KRC and remote pc
     * We choose by convention to trigger fri_start() in
     * the KRL program if $FRI_FRM_INT[1] == 1
     */
    bool configureHook(){
        //initialize the arrays that will be send to KRL
        for(int i=0; i<16; ++i){
            fri_to_krl.intData[i]=0;
            fri_to_krl.realData[i]=0.0;
        }

        //We chose to put 1 on the $FRI_FRM_INT[1] to trigger the fri_start()
        fri_to_krl.intData[0]=1;

        return true;
    }

    bool startHook(){
        //Send arrays to KRC
        port_fri_to_krl.write(fri_to_krl);
        return doStart();
    }

    virtual bool doStart() = 0;
    virtual void updateHook() = 0;
    virtual void stopHook() = 0;
    virtual void cleanupHook() = 0;

};
#endif
