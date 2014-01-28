// Copyright 2014 ISIR-CNRS
// Author: Sovannara Hak

#include "friExampleAbstract.hpp"

FriExampleAbstract::FriExampleAbstract(std::string const& name) : RTT::TaskContext(name){
    this->addPort("fromFRI", port_fri_to_krl);
    this->addPort("toFRI", port_fri_frm_krl);

    this->addPort("RobotState_i", iport_robot_state);
    this->addPort("FRIJointState_i", iport_fri_joint_state);
    this->addPort("CartesianPosition_i", iport_cart_pos);
    this->addPort("CartesianPositionFrame_i", iport_cart_frame);
    this->addPort("JointState_i", iport_joint_state);
    this->addPort("CartesianWrench_i", iport_cart_wrench);
    this->addPort("Jacobian_i", iport_jacobian);
    this->addPort("MassMatrix_i", iport_mass_matrix);

    this->addPort("JointPositions_o", oport_joint_position);
    this->addPort("JointVelocities_o", oport_joint_velocities);
    this->addPort("JointTorques_o", oport_joint_efforts);
    this->addPort("JointImpedance_o", oport_joint_impedance);
    this->addPort("CartesianPosition_o", oport_cartesian_pose);
    this->addPort("CartesianVelocity_o", oport_cartesian_twist);
    this->addPort("CartesianWrench_o", oport_cartesian_wrench);
    this->addPort("CartesianImpedance_o", oport_cartesian_impedance);

    this->addOperation("getFRIMode", &FriExampleAbstract::getFRIMode, this, RTT::OwnThread);

    this->addOperation("setControlStrategy", &FriExampleAbstract::setControlStrategy, this, RTT::OwnThread);

    this->addOperation("friStart", &FriExampleAbstract::friStart, this, RTT::OwnThread);
    this->addOperation("friStop", &FriExampleAbstract::friStop, this, RTT::OwnThread);
    this->addOperation("stopKrlScript", &FriExampleAbstract::stopKrlScript, this, RTT::OwnThread);

    LWRDOF = 7;
}

FriExampleAbstract::~FriExampleAbstract(){
}

bool FriExampleAbstract::configureHook(){
    //initialize the arrays that will be send to KRL
    for(int i=0; i<16; ++i){
        fri_to_krl.intData[i]=0;
        fri_to_krl.realData[i]=0.0;
    }

    //We chose to put 1 on the $FRI_FRM_INT[1] to trigger the fri_start()
    //In KRL, index starts at 1
    fri_to_krl.intData[0]=1;
    //Set control strategy to joint position
    fri_to_krl.intData[1]=10;
    controlMode = 10;

    return true;
}

bool FriExampleAbstract::startHook(){
    return doStart();
}

bool FriExampleAbstract::doStart(){
    friStart();
    return true;
}

void FriExampleAbstract::stopHook(){
    doStop();
}

void FriExampleAbstract::doStop(){
    stopKrlScript();
}

void FriExampleAbstract::cleanupHook(){}

void FriExampleAbstract::setControlStrategy(int mode){
    if(mode != 10 && mode != 20 && mode != 30){
        std::cout << "Please set a valid control mode: " << std::endl;
        std::cout << "10: Joint position" << std::endl;
        std::cout << "20: Cartesian stiffness" << std::endl;
        std::cout << "30: Joint stiffness" << std::endl;
        return;
    }
    else{
        fri_to_krl.intData[1] = mode;
        controlMode = mode;
        port_fri_to_krl.write(fri_to_krl);
    }
}

bool FriExampleAbstract::requiresControlMode(int modeRequired){
    if (controlMode == modeRequired){
        return true;
    }
    else{
        std::cout << "Cannot proceed, current control mode is " << controlMode
            << " required control mode is " << modeRequired << std::endl;
        return false;
    }
}

void FriExampleAbstract::getFRIMode(){
    RTT::FlowStatus fri_frm_krl_fs = port_fri_frm_krl.read(fri_frm_krl);
    if(fri_frm_krl_fs == RTT::NewData){
        if(fri_frm_krl.intData[1] == 1){
            std::cout << "FRI in Command Mode" << std::endl;
        }
        else if(fri_frm_krl.intData[1] == 2){
            std::cout << "FRI in Monitor Mode" << std::endl;
        }
    }
    else{
        std::cout << "Cannot read FRI Mode" << std::endl;
    }
}

void FriExampleAbstract::friStop(){
    //Put 2 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=2;
    port_fri_to_krl.write(fri_to_krl);
    return;
}

void FriExampleAbstract::friStart(){
    //Put 1 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=1;
    port_fri_to_krl.write(fri_to_krl);
    return;
}

void FriExampleAbstract::stopKrlScript(){
    //Put 3 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=3;
    port_fri_to_krl.write(fri_to_krl);
}
