// Copyright (C) 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExample.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <boost/foreach.hpp>
#include <kdl/frames_io.hpp>

FriExample::FriExample(std::string const& name) : FriExampleAbstract(name){
    this->addPort("RobotState_i", iport_robot_state);
    this->addPort("FRIJointState_i", iport_fri_joint_state);
    this->addPort("CartesianPosition_i", iport_cart_pos);
    this->addPort("CartesianPositionFrame_i", iport_cart_frame);
    this->addPort("JointState_i", iport_joint_state);
    this->addPort("CartesianWrench_i", iport_cart_wrench);
    this->addPort("Jacobian_i", iport_jacobian);

    this->addOperation("getFRIJointState", &FriExample::getFRIJointState, this, RTT::OwnThread);
    this->addOperation("getCartesianPosition", &FriExample::getCartesianPosition, this, RTT::OwnThread);
    this->addOperation("getCartesianFrame", &FriExample::getCartesianFrame, this, RTT::OwnThread);
    this->addOperation("getRobotState", &FriExample::getRobotState, this, RTT::OwnThread);
    this->addOperation("getJointState", &FriExample::getJointState, this, RTT::OwnThread);
    this->addOperation("getCartesianWrench", &FriExample::getCartesianWrench, this, RTT::OwnThread);
    this->addOperation("getJacobian", &FriExample::getJacobian, this, RTT::OwnThread);

    this->addOperation("setControlStrategy", &FriExample::setControlStrategy, this, RTT::OwnThread);
}

FriExample::~FriExample(){
}

void FriExample::updateHook(){
}

void FriExample::getFRIJointState(){
    lwr_fri::FriJointState fri_joint_state_data;
    RTT::FlowStatus fri_jointStateFS = iport_fri_joint_state.read(fri_joint_state_data);

    if(fri_jointStateFS == RTT::NewData){
        std::cout << "Measured Joint configuration" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.msrJntPos){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Commanded Joint configuration before FRI" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.cmdJntPos){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Commanded Joint configuration FRI offset" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.cmdJntPosFriOffset){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Measured Joint torque actuator" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.msrJntTrq){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Estimated External Joint torque sensor" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.estExtJntTrq){
            std::cout << f << " ";
        }
        std::cout << std::endl;

        std::cout << "Gravity compensation" << std::endl;
        BOOST_FOREACH(float f, fri_joint_state_data.gravity){
            std::cout << f << " ";
        }
        std::cout << std::endl;
    }
}

void FriExample::getCartesianPosition(){
    geometry_msgs::Pose cartPosData;
    RTT::FlowStatus cartPosFS = iport_cart_pos.read(cartPosData);

    if(cartPosFS == RTT::NewData){
        std::cout << "Position end effector [x, y, z]:" << std::endl;
        std::cout << "[" << cartPosData.position.x
                  << ", " << cartPosData.position.y 
                  << ", " << cartPosData.position.z
                  << "]" << std::endl;
        std::cout << "Orientation end effector [x, y, z, w]:" << std::endl;
        std::cout << "[" << cartPosData.orientation.x
                  << ", " << cartPosData.orientation.y
                  << ", " << cartPosData.orientation.z
                  << ", " << cartPosData.orientation.w
                  << "]" << std::endl;
    }
}

void FriExample::getCartesianFrame(){
    KDL::Frame cart_frame_data;
    RTT::FlowStatus cartFrameFS = iport_cart_frame.read(cart_frame_data);
    if(cartFrameFS == RTT::NewData){
        std::cout << "Cartesian Frame end effector" << std::endl;
        std::cout << cart_frame_data << std::endl;
    }

}

void FriExample::getRobotState(){
    tFriRobotState robot_state_data;
    RTT::FlowStatus robotStateFS = iport_robot_state.read(robot_state_data);
    if(robotStateFS == RTT::NewData){
        std::cout << "Power state drive" << std::endl;
        std::cout << robot_state_data.power << std::endl;
        std::cout << "Selected control strategy" << std::endl;
        std::cout << robot_state_data.control << std::endl;
        std::cout << "Drive error" << std::endl;
        std::cout << robot_state_data.error << std::endl;
        std::cout << "Temperature of drives" << std::endl;
        for(unsigned int i=0; i<7; i++)
            std::cout << robot_state_data.temperature[i] << " ";
        std::cout << std::endl;
    }
}

void FriExample::getJointState(){
    sensor_msgs::JointState joint_state_data;
    RTT::FlowStatus joint_state_fs = iport_joint_state.read(joint_state_data);
    if(joint_state_fs == RTT::NewData){
        std::cout << "Joint State" << std::endl;
        //joint_state_data.position is a std::vector
        BOOST_FOREACH(double d, joint_state_data.position){
            std::cout << d << " " << std::endl;
        }
        std::cout << "Joint effort" << std::endl;
        BOOST_FOREACH(double d, joint_state_data.effort){
            std::cout << d << " " << std::endl;
        }
    }

}

void FriExample::getCartesianWrench(){
    geometry_msgs::Wrench cart_wrench_data;
    RTT::FlowStatus cart_wrench_fs = iport_cart_wrench.read(cart_wrench_data);
    if(cart_wrench_fs == RTT::NewData){
        std::cout << "Cartesian Force at end effector" << std::endl;
        std::cout << cart_wrench_data.force << std::endl;
        std::cout << "Cartesian torque at end effector" << std::endl;
        std::cout << cart_wrench_data.torque << std::endl;
    }

}

void FriExample::getJacobian(){
    KDL::Jacobian jacobian_data;
    RTT::FlowStatus jacobian_fs = iport_jacobian.read(jacobian_data);
    if(jacobian_fs == RTT::NewData){
        std::cout << "Jacobian: " << std::endl;
        std::cout << jacobian_data.data << std::endl;
    }

}

void FriExample::setControlStrategy(int mode){
    if(mode != 10 || mode != 20 || mode != 30){
        std::cout << "Please set a valid control mode: " << std::endl;
        std::cout << "10: Joint position" << std::endl;
        std::cout << "20: Cartesian stiffness" << std::endl;
        std::cout << "30: Joint stiffness" << std::endl;
        return;
    }
    else{
        fri_to_krl.intData[1] = mode;
        controlMode = mode;
    }
}

ORO_CREATE_COMPONENT(FriExample)
