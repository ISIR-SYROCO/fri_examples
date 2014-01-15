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
        //In KRL, index starts at 1
        fri_to_krl.intData[0]=1;

        return true;
    }

    /** @brief Orocos Start Hook
     * Send arrays to KRC and call doStart()
     */
    bool startHook(){
        //Send arrays to KRC
        port_fri_to_krl.write(fri_to_krl);
        return doStart();
    }

    /** @brief To implement if specific things have to be done when starting the component
     */
    virtual bool doStart(){
        return true;
    }

    /** @brief Orocos Update hook
     */
    virtual void updateHook() = 0;

    /** @brief Orocos stop hook
     * Call doStop(), then put 2 in $FRI_FRM_INT[1]
     * which by our convention trigger fri_stop() in KRL program
     */
    void stopHook(){
        doStop();
        //Put 2 in $FRI_FRM_INT[1] to trigger fri_stop()
        fri_to_krl.intData[0]=2;
        port_fri_to_krl.write(fri_to_krl);
    }

    /** @brief To implement if specific things have to be done when stoping the component
     */
    virtual void doStop(){}

    /** @brief Orocos Cleanup hook
     */
    virtual void cleanupHook(){};

};
#endif
