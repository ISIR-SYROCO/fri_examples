// Copyright 2014 ISIR-CNRS
// Author: Sovannara Hak

#ifndef OROCOS_FRI_EXAMPLE_ABSTRACT_HPP
#define OROCOS_FRI_EXAMPLE_ABSTRACT_HPP

#include <rtt/RTT.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <lwr_fri/typekit/Types.hpp>
#include <kuka_lwr_fri/friComm.h>

#include <geometry_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>

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

    RTT::InputPort<lwr_fri::FriJointState>  iport_fri_joint_state;
    RTT::InputPort<geometry_msgs::Pose>     iport_cart_pos;
    RTT::InputPort<KDL::Frame>              iport_cart_frame;
    RTT::InputPort<tFriRobotState>          iport_robot_state;
    RTT::InputPort<sensor_msgs::JointState> iport_joint_state;
    RTT::InputPort<geometry_msgs::Wrench>   iport_cart_wrench;
    RTT::InputPort<KDL::Jacobian>           iport_jacobian;
    RTT::InputPort< Eigen::Matrix<double, 7, 7> >        iport_mass_matrix;

    RTT::OutputPort<motion_control_msgs::JointPositions>  oport_joint_position;
    RTT::OutputPort<motion_control_msgs::JointVelocities> oport_joint_velocities;
    RTT::OutputPort<motion_control_msgs::JointEfforts>    oport_joint_efforts;
    RTT::OutputPort<lwr_fri::FriJointImpedance>           oport_joint_impedance;
    RTT::OutputPort<geometry_msgs::Pose>                  oport_cartesian_pose;
    RTT::OutputPort<geometry_msgs::Twist>                 oport_cartesian_twist;
    RTT::OutputPort<geometry_msgs::Wrench>                oport_cartesian_wrench;
    RTT::OutputPort<lwr_fri::CartesianImpedance>          oport_cartesian_impedance;

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

    /** @brief Ask KRL exit
     */
    void stopKrlScript();

    /** @brief Initialize the command that will be send to the robot
     */
    void initializeCommand();

};
#endif
