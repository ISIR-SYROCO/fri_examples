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

    /** @brief Store current control mode
     * controlMode = 10  : Joint position
     * controlMode = 20  : Cartesian stiffness
     * controlMode = 30  : Joint stiffness
     */
    int controlMode;

    unsigned int LWRDOF;

    /**
     * Output port to send shared arrays to the KRC
     */
    RTT::OutputPort<tFriKrlData> port_fri_to_krl;

    /**
     * Input port to read shared arrays from the KRC
     */
    RTT::InputPort<tFriKrlData> port_fri_frm_krl;

    FriExampleAbstract(std::string const& name);
    ~FriExampleAbstract();

    /** @brief Orocos Configure Hook
     * Initialization of the shared array between
     * KRC and remote pc
     * We choose by convention to trigger fri_start() in
     * the KRL program if $FRI_FRM_INT[1] == 1
     */
    bool configureHook();

    /** @brief Orocos Start Hook
     * Send arrays to KRC and call doStart()
     */
    bool startHook();

    /** @brief To implement if specific things have to be done when starting the component
     */
    virtual bool doStart();

    /** @brief Orocos Update hook
     */
    virtual void updateHook() = 0;

    /** @brief Orocos stop hook
     * Call doStop(), then put 2 in $FRI_FRM_INT[1]
     * which by our convention trigger fri_stop() in KRL program
     */
    void stopHook();

    /** @brief To implement if specific things have to be done when stoping the component
     */
    virtual void doStop();

    /** @brief Orocos Cleanup hook
     */
    virtual void cleanupHook();

    /** @brief Set control strategy
     */
    void setControlStrategy(int mode);

    /** @brief Check if the selected control mode is the required one
     *  @param modeRequired : the required mode
     *  @return True : if the current mode match the required one
     */
    bool requiresControlMode(int modeRequired);

    /** @brief Get the FRI Mode
     */
    void getFRIMode();

    /** @brief Ask KRL script for a friStop()
     */
    void friStop();

    /** @brief Ask KRL script for a friStart()
     */
    void friStart();

};
#endif
