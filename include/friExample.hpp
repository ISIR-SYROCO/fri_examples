// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#ifndef FRI_EXAMPLE__HPP
#define FRI_EXAMPLE__HPP

#include "friExampleAbstract.hpp"

#include <string>

#include <rtt/RTT.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <lwr_fri/typekit/Types.hpp>
#include <kuka_lwr_fri/friComm.h>

#include <geometry_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>

class FriExample : public FriExampleAbstract{
    public:
        FriExample(std::string const& name);
        ~FriExample();

        void updateHook();

        void getFRIJointState();
        void getCartesianPosition();
        void getCartesianFrame();
        void getRobotState();
        void getJointState();
        void getCartesianWrench();
        void getJacobian();

        RTT::InputPort<lwr_fri::FriJointState>  iport_fri_joint_state;
        RTT::InputPort<geometry_msgs::Pose>     iport_cart_pos;
        RTT::InputPort<KDL::Frame>              iport_cart_frame;
        RTT::InputPort<tFriRobotState>          iport_robot_state;
        RTT::InputPort<sensor_msgs::JointState> iport_joint_state;
        RTT::InputPort<geometry_msgs::Wrench>   iport_cart_wrench;
        RTT::InputPort<KDL::Jacobian>           iport_jacobian;

        RTT::OutputPort<motion_control_msgs::JointPositions>  oport_joint_position;
        RTT::OutputPort<motion_control_msgs::JointVelocities> oport_joint_velocities;
        RTT::OutputPort<motion_control_msgs::JointEfforts>    oport_joint_efforts;
        RTT::OutputPort<lwr_fri::FriJointImpedance>           oport_joint_impedance;
        RTT::OutputPort<geometry_msgs::Pose>                  oport_cartesian_pose;
        RTT::OutputPort<geometry_msgs::Twist>                 oport_cartesian_twist;
        RTT::OutputPort<geometry_msgs::Wrench>                oport_cartesian_wrench;
        RTT::OutputPort<lwr_fri::CartesianImpedance>          oport_cartesian_impedance;
};

#endif
