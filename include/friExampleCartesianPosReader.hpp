// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#ifndef FRI_EXAMPLE_CARTESIAN_POS_READER_HPP
#define FRI_EXAMPLE_CARTESIAN_POS_READER_HPP

#include <rtt/RTT.hpp>
#include "friExampleAbstract.hpp"
#include <kuka_lwr_fri/friComm.h>
#include <lwr_fri/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <string>

class FriExampleCartesianPosReader : public FriExampleAbstract{
    public:
        FriExampleCartesianPosReader(std::string const& name);
        ~FriExampleCartesianPosReader();

        void updateHook();

        RTT::InputPort<geometry_msgs::Pose> iport_cart_pos;
};

#endif
