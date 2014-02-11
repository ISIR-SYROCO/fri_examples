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
    friMode = 1;
    controlModeUpdated = 1;

    return true;
}

bool FriExampleAbstract::startHook(){
    initializeCommand();
    return doStart();
}

bool FriExampleAbstract::doStart(){
    friStart();
    return true;
}

void FriExampleAbstract::stopHook(){
    //Reset all commands sent to the fri
    initializeCommand();
    doStop();
}

void FriExampleAbstract::doStop(){
    friStop();
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
        controlModeUpdated = 1;
    }
}

bool FriExampleAbstract::requiresControlMode(int modeRequired){
    if (controlMode == modeRequired){
        return true;
    }
    else{
        if(controlModeUpdated == 1){
            std::cout << "Cannot proceed, current control mode is " << controlMode
                << " required control mode is " << modeRequired << std::endl;
            controlModeUpdated = 0;
        }
        return false;
    }
}

void FriExampleAbstract::getFRIMode(){
    RTT::FlowStatus fri_frm_krl_fs = port_fri_frm_krl.read(fri_frm_krl);
    if(fri_frm_krl_fs == RTT::NewData){
        if(fri_frm_krl.intData[0] == 1){
            friMode = 1;
            std::cout << "FRI in Command Mode" << std::endl;
        }
        else if(fri_frm_krl.intData[0] == 2){
            friMode = 2;
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
    friMode = 2;
    return;
}

void FriExampleAbstract::friStart(){
    //Put 1 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=1;
    port_fri_to_krl.write(fri_to_krl);
    friMode = 1;
    return;
}

void FriExampleAbstract::stopKrlScript(){
    //Back to position control
    setControlStrategy(10);
    //Put 3 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=3;
    port_fri_to_krl.write(fri_to_krl);
}

void FriExampleAbstract::initializeCommand(){
    //Get current joint position and set it as desired position
    if (oport_joint_position.connected()){ 
        lwr_fri::FriJointState fri_joint_state_data;
        RTT::FlowStatus fri_joint_state_fs = iport_fri_joint_state.read(fri_joint_state_data);
        if (fri_joint_state_fs == RTT::NewData){
            motion_control_msgs::JointPositions joint_position_command;
            joint_position_command.positions.assign(7, 0.0);
            for (unsigned int i = 0; i < LWRDOF; i++){
                joint_position_command.positions[i] = fri_joint_state_data.msrJntPos[i];
            }
            oport_joint_position.write(joint_position_command);
        }
    }

    if(oport_cartesian_pose.connected()){
        //Get cartesian position and set it as desired position
        geometry_msgs::Pose cartPosData;
        RTT::FlowStatus cart_pos_fs = iport_cart_pos.read(cartPosData);
        if (cart_pos_fs == RTT::NewData){
            oport_cartesian_pose.write(cartPosData);
        }
    }
    
    if(oport_cartesian_wrench.connected()){
        //Send 0 torque and force
        geometry_msgs::Wrench cart_wrench_command;
        cart_wrench_command.force.x = 0.0;
        cart_wrench_command.force.y = 0.0;
        cart_wrench_command.force.z = 0.0;
        cart_wrench_command.torque.x = 0.0;
        cart_wrench_command.torque.y = 0.0;
        cart_wrench_command.torque.z = 0.0;

        oport_cartesian_wrench.write(cart_wrench_command);
    }

    if (oport_joint_efforts.connected()){
        //Send 0 joint torque
        motion_control_msgs::JointEfforts joint_eff_command;
        joint_eff_command.efforts.assign(LWRDOF, 0.0); 

        oport_joint_efforts.write(joint_eff_command);
    }

}
