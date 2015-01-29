// Copyright 2014 ISIR-CNRS
// Author: Guillaume Hamon

#ifndef OROCOS_FRI_RTNET_EXAMPLE_ABSTRACT_HPP
#define OROCOS_FRI_RTNET_EXAMPLE_ABSTRACT_HPP

#include <rtt/RTT.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <lwr_fri/typekit/Types.hpp>
#include <kuka_lwr_fri/friComm.h>

#include <geometry_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>

#include <rtt/Attribute.hpp>

class FriRTNetExampleAbstract : public RTT::TaskContext{
  public:
	RTT::TaskContext* peer;


    /**
     * @brief Shared arrays from the remote pc to the KRC
     */
    tFriKrlData fri_to_krl;

    /**
     * @brief Shared arrays from the KRC to the remote pc
     */
    tFriKrlData fri_frm_krl;

    /** @brief Store current control mode
     * controlMode = 10  : Joint position
     * controlMode = 20  : Cartesian stiffness
     * controlMode = 30  : Joint stiffness
     */
    int controlMode;

    unsigned int LWRDOF;
    unsigned int fri_desired_mode;

    /**
     * @brief Attribute to send shared arrays to the KRC
     */
  	RTT::Attribute<tFriKrlData> m_toKRL;

    /**
     * @brief Attribute to read shared arrays from the KRC
     */
	RTT::Attribute<tFriKrlData> m_fromKRL;

	/**
	 * @brief Property to get and set the control_mode (1 to 7)
	 */
	RTT::Property<int> control_mode_prop;

    RTT::InputPort<tFriRobotState> iport_robot_state;
    RTT::InputPort<tFriIntfState>  iport_Fri_state;
    RTT::InputPort< std::vector<double> > iport_msr_joint_pos;
    RTT::InputPort< std::vector<double> > iport_cmd_joint_pos;
    RTT::InputPort< std::vector<double> > iport_cmd_joint_pos_fri_offset;
    RTT::InputPort< geometry_msgs::Pose > iport_cart_pos;
    RTT::InputPort< geometry_msgs::Pose > iport_cmd_cart_pos;
    RTT::InputPort< geometry_msgs::Pose > iport_cmd_cart_pos_fri_offset;
    RTT::InputPort< std::vector<double> > iport_msr_joint_vel;
    RTT::InputPort< std::vector<double> > iport_msr_joint_trq;
    RTT::InputPort< std::vector<double> > iport_est_ext_joint_trq;
    RTT::InputPort< geometry_msgs::Wrench > iport_cart_wrench;
    RTT::InputPort< std::string > iport_events;

    RTT::InputPort<KDL::Jacobian> jacobianPort;
    RTT::InputPort<std::vector<double> > gravityPort;
    RTT::InputPort< Eigen::Matrix<double, 7, 7> > iport_mass_matrix;

    RTT::OutputPort< std::vector<double> > oport_joint_position;
    RTT::OutputPort< std::vector<double> > oport_joint_velocities;
    RTT::OutputPort< std::vector<double> > oport_add_joint_trq;
    RTT::OutputPort<geometry_msgs::Pose> oport_cartesian_pose;
    RTT::OutputPort<geometry_msgs::Twist> oport_cartesian_twist;
    RTT::OutputPort<geometry_msgs::Wrench> oport_cartesian_wrench;
    RTT::OutputPort<lwr_fri::FriJointImpedance> oport_joint_impedance;

    FriRTNetExampleAbstract(std::string const& name);
    ~FriRTNetExampleAbstract();

    std::vector<double> mass_matrix;

    std::vector<double> estExtJntTrq;
    std::vector<double> estExtTcpWrench;

	double current_load;

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

    /** @brief define the lwr_fri peer name to share attributes toKrl and fromKrl
     * */
    void setPeer(std::string name);

    /** @brief Set control strategy
	 *  1 : joint position
	 *  2 : joint velocity
	 *  3 : joint torque
	 *  4 : cartesian position
	 *  5 : cartesian force
	 *  6 : cartesian twist
	 *  7 : joint position and torque
	 *  Then $FRI_*_INT[2] is set to the apprioriate value:
	 *  10 if control strategy 1 or 2
	 *  20 if control strategy 4, 5 or 6
	 *  30 if control strategy 3 or 7
     */
    void setControlStrategy(int mode);

    /** @brief Select the tool defined on the KRC
	 *  The tool number is stored in $FRI_*_INT[3]
     */
    void setTool(int toolNumber);

    /** @brief Set new load
	 *  The value of the load is stored in $FRI_*_REA[1]
	 */
	void setLoad(float load);

    /** @brief Get current load (tool weight)
	 *  The value of the load is stored in $FRI_*_REA[1]
	 */
	void getLoad();

    /** @brief Check if the selected control mode is the required one
     *  @param modeRequired : the required mode
     *  @return True : if the current mode match the required one
     */
    bool requiresControlMode(int modeRequired);

    /** @brief Get the FRI Mode
     *  @return : The value of the FRI_STATE enum
     */
    FRI_STATE getFRIMode();

    /** @brief Ask KRL script for a friStop()
     */
    void friStop();

    /** @brief Ask KRL script for a friStart()
     */
    void friStart();

    /** @brief Reset the array shared with the KRC
     */
    void friReset();

    /** @brief Ask KRL exit
     */
    void stopKrlScript();

    /** @brief Initialize the command that will be send to the robot
     */
    void initializeCommand();

    /** @brief Return the cartesian position of the tool center point in the robot base frame
     */
    std::vector<double> getCartPos();

    /** @brief Return the Jacobian
     */
    std::vector<double> getJacobian();

    /** @brief Return the Mass Matrix
     */
    std::vector<double> getMassMatrix();

    /** @brief Return the gravity
     */
    std::vector<double> getGravity();

    /** @brief Return the current configuration of the robot
     */
    std::vector<double> getQ();

    /** @brief Return the estimated external joint torque
     */
    std::vector<double> getEstExtJntTrq();

    /** @brief Return the estimated external tool center point wrench
     */
    std::vector<double> getEstExtTcpWrench();

    bool connectIRobotState();
    bool connectIFriState();
    bool connectIMsrJntPos();
    bool connectICmdJntPos();
    bool connectICmdJntPosFriOffset();
    bool connectIMsrCartPos();
    bool connectICmdCartPos();
    bool connectICmdCartPosFriOffset();
    bool connectIMsrJntVel();
    bool connectIMsrJntTrq();
    bool connectIEstExtJntTrq();
    bool connectIEstExtTcpWrench();
    bool connectIEvents();
    bool connectIMassMatrix();
    bool connectIJacobian();
    bool connectIGravity();
    

    /** @brief Connect joint position output ports with lwr_fri component
     */
    bool connectOJointPosition();

    /** @brief Connect joint velocities output ports with lwr_fri component
     */
    bool connectOJointVelocities();

    /** @brief Connect joint torque output ports with lwr_fri component
     */
    bool connectOJointTorque();

    /** @brief Connect cartesian pose output ports with lwr_fri component
     */
    bool connectOCartesianPose();

    /** @brief Connect cartesian pose output ports with lwr_fri component
     */
    bool connectOCartesianTwist();

    bool connectOCartesianWrench();
    bool connectODesJntImpedance();

    void disconnectPort(std::string portname);

    /** @brief Send Joint position in radians
     */
    void sendJointPosition(std::vector<double> &qdes);

    /** @brief Send Joint velocities in radians/s
     */
    void sendJointVelocities(std::vector<double> &qdotdes);

    /** @brief Send additionnal joint torque in Nm
     */
    void sendAddJointTorque(std::vector<double> &tau);

    /** @brief Send desired tool center point cartesian pose: x, y, z(m), qw, qx, qy, qz
     */
    void sendCartesianPose(std::vector<double> &pose);

    /** @brief Send desired tool center point cartesian twist: vx, vy, vz, wx, wy, wz
     */
    void sendCartesianTwist(std::vector<double> &twist);

};
#endif
