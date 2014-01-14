// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#ifndef FRI_EXAMPLE_JOINT_READER_HPP
#define FRI_EXAMPLE_JOINT_READER_HPP

#include <rtt/RTT.hpp>
#include "friExampleAbstract.hpp"
#include <kuka_lwr_fri/friComm.h>
#include <lwr_fri/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <string>

class FriExampleJointReader : public FriExampleAbstract{
    public:
        FriExampleJointReader(std::string const& name);
        ~FriExampleJointReader();
        bool doStart();
        void updateHook();
        void stopHook();
        void cleanupHook();

        RTT::InputPort<lwr_fri::FriJointState> iport_joint_state;

};

#endif
