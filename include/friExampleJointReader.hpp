// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#ifndef FRI_EXAMPLE_JOINT_READER_HPP
#define FRI_EXAMPLE_JOINT_READER_HPP

#include <rtt/RTT.hpp>
#include "friExampleAbstract.hpp"
#include <kuka_lwr_fri/friComm.h>
#include <lwr_fri/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <boost/array.hpp>
#include <string>

class FriExampleJointReader : public FriExampleAbstract{
    public:
        FriExampleJointReader(std::string const& name);
        ~FriExampleJointReader();

        void updateHook();

        RTT::InputPort<lwr_fri::FriJointState> iport_joint_state;
    
    private:
        void printData(boost::array<float, 7> &vec);
};

#endif
