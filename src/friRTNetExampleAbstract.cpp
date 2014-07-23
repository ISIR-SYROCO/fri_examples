// Copyright 2014 ISIR-CNRS
// Author: Guillaume Hamon

#include "friRTNetExampleAbstract.hpp"


FriRTNetExampleAbstract::FriRTNetExampleAbstract(std::string const& name) : RTT::TaskContext(name){
    this->addPort("RobotState_i", iport_robot_state);
    this->addPort("FriState_i", iport_Fri_state);
    this->addPort("MsrJntPos_i", iport_msr_joint_pos);
    this->addPort("CmdJntPos_i", iport_cmd_joint_pos);
    this->addPort("CmdJntPosFriOffset_i", iport_cmd_joint_pos_fri_offset);
    this->addPort("MsrCartPos_i", iport_cart_pos);
    this->addPort("CmdCartPos_i", iport_cmd_cart_pos);
    this->addPort("CmdCartPosFriOffset_i", iport_cmd_cart_pos_fri_offset);
    this->addPort("msrJntVel_i", iport_msr_joint_vel);
    this->addPort("MsrJntTrq_i", iport_msr_joint_trq);
    this->addPort("EstExtJntTrq_i", iport_est_ext_joint_trq);
    this->addPort("EstExtTcpWrench_i", iport_cart_wrench);
    this->addPort("Events_i", iport_events);
    this->addPort("MassMatrix_i", iport_mass_matrix);
    this->addPort("Jacobian_i", jacobianPort);
    this->addPort("gravity_i", gravityPort);


    this->addPort("JointPositions_o", oport_joint_position);
    this->addPort("JointVelocities_o", oport_joint_velocities);
    this->addPort("JointTorques_o", oport_add_joint_trq);
    this->addPort("CartesianPosition_o", oport_cartesian_pose);
    this->addPort("CartesianVelocity_o", oport_cartesian_twist);
    this->addPort("CartesianWrench_o", oport_cartesian_wrench);
    this->addPort("desJntImpedance_o",oport_joint_impedance);

    this->addOperation("getFRIMode", &FriRTNetExampleAbstract::getFRIMode, this, RTT::OwnThread);
    this->addOperation("setPeer", &FriRTNetExampleAbstract::setPeer, this, RTT::OwnThread);

    this->addOperation("setControlStrategy", &FriRTNetExampleAbstract::setControlStrategy, this, RTT::OwnThread);

    this->addOperation("friStart", &FriRTNetExampleAbstract::friStart, this, RTT::OwnThread);
    this->addOperation("friStop", &FriRTNetExampleAbstract::friStop, this, RTT::OwnThread);
    this->addOperation("friReset", &FriRTNetExampleAbstract::friReset, this, RTT::OwnThread);
    this->addOperation("stopKrlScript", &FriRTNetExampleAbstract::stopKrlScript, this, RTT::OwnThread);
    this->addOperation("getCartPos", &FriRTNetExampleAbstract::getCartPos, this, RTT::OwnThread);
    this->addOperation("getJacobian", &FriRTNetExampleAbstract::getJacobian, this, RTT::OwnThread);

    this->addOperation("sendJointPosition", &FriRTNetExampleAbstract::sendJointPosition, this, RTT::OwnThread);
    this->addOperation("sendJointVelocities", &FriRTNetExampleAbstract::sendJointPosition, this, RTT::OwnThread);
    this->addOperation("sendAddJointTorque", &FriRTNetExampleAbstract::sendAddJointTorque, this, RTT::OwnThread);
    this->addOperation("connectOJointPosition", &FriRTNetExampleAbstract::connectOJointPosition, this, RTT::OwnThread);
    this->addOperation("connectOJointVelocities", &FriRTNetExampleAbstract::connectOJointVelocities, this, RTT::OwnThread);
    this->addOperation("connectOJointTorque", &FriRTNetExampleAbstract::connectOJointTorque, this, RTT::OwnThread);
    this->addOperation("connectOCartesianPose", &FriRTNetExampleAbstract::connectOCartesianPose, this, RTT::OwnThread);

    LWRDOF = 7;
}

FriRTNetExampleAbstract::~FriRTNetExampleAbstract(){
}

bool FriRTNetExampleAbstract::configureHook(){
	setPeer("lwr");
    //initialize the arrays that will be send to KRL
    for(int i=0; i<16; ++i){
        fri_to_krl.intData[i]=0;
        fri_to_krl.realData[i]=0.0;
    }

    if (!iport_events.connected()){
        std::cout << this->getName() << ".Events_i port not connected, cannot configure" << std::endl;
        return false;
    }

    return true;
}

bool FriRTNetExampleAbstract::startHook(){
    initializeCommand();
    return doStart();
}

bool FriRTNetExampleAbstract::doStart(){
    friStart();
    return true;
}

void FriRTNetExampleAbstract::stopHook(){
    //Reset all commands sent to the fri
    initializeCommand();
    doStop();
}

void FriRTNetExampleAbstract::doStop(){
    friStop();
    //stopKrlScript();
}

void FriRTNetExampleAbstract::cleanupHook(){}

//define peer (lwr_fri component) and get access to attributes and properties
void FriRTNetExampleAbstract::setPeer(std::string name){
    peer = getPeer(name);
    assert(peer);
    m_toFRI = peer->attributes()->getAttribute("toKRL");
    m_fromFRI= peer->attributes()->getAttribute("fromKRL");
    control_mode_prop = peer->properties()->getProperty("control_mode");

}

void FriRTNetExampleAbstract::setControlStrategy(int mode){
    if(mode != 1 && mode != 2 && mode != 3 && mode != 4 && mode != 5 && mode != 6 && mode != 7){
        std::cout << "Please set a valid control mode: " << std::endl;
        std::cout << "1: Joint position" << std::endl;
        std::cout << "2: Joint velocity" << std::endl;
        std::cout << "3: Joint torque" << std::endl; 
        std::cout << "4: Cartesian position" << std::endl;
        std::cout << "5: Cartesian force" << std::endl;
        std::cout << "6: Cartesian Twist" << std::endl;
        std::cout << "7: Joint Position and torque for objects picking" << std::endl; //perform joint impedance control (position + torque compensation of the load)
        return;
    }
    else{
        if (mode == 1 || mode == 2){
            fri_to_krl.intData[1] = 10;
            controlMode = 10; // joint position control
        }
        else if (mode == 3 || mode == 7){
            fri_to_krl.intData[1] = 30;
            controlMode = 30; // joint impedance control
            control_mode_prop.set(7);
        }
        else if (mode == 4 || mode == 5 || mode == 6){
            fri_to_krl.intData[1] = 20;
            controlMode = 20; // cartesian impedance control
        }
        m_toFRI.set(fri_to_krl);
    }
}

void FriRTNetExampleAbstract::setTool(int toolNumber){
	fri_to_krl.intData[2] = toolNumber;
	m_toFRI.set(fri_to_krl);
}


bool FriRTNetExampleAbstract::requiresControlMode(int modeRequired){
    if (controlMode == modeRequired){
        return true;
    }
    else{
        std::cout << "Cannot proceed, current control mode is " << controlMode
            << " required control mode is " << modeRequired << std::endl;
        return false;
    }
}

FRI_STATE FriRTNetExampleAbstract::getFRIMode(){
    fri_frm_krl = m_fromFRI.get();
    if(fri_frm_krl.intData[0] == 1){
        std::cout << "FRI in Command Mode" << std::endl;
        return FRI_STATE_CMD;
    }else{
        if(fri_frm_krl.intData[0] == 2){
            std::cout << "FRI in Monitor Mode" << std::endl;
            return FRI_STATE_MON;
        }else{
            std::cout << "Cannot read FRI Mode" << std::endl;
            return FRI_STATE_INVALID;
        }
    }
}

void FriRTNetExampleAbstract::friStop(){

    //Put 2 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=2;
    m_toFRI.set(fri_to_krl);

    return;
}

void FriRTNetExampleAbstract::friStart(){

    //Put 1 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=1;
    m_toFRI.set(fri_to_krl);

    return;
}

void FriRTNetExampleAbstract::friReset(){
    //initialize the arrays that will be send to KRL
    for(int i=0; i<16; ++i){
        fri_to_krl.intData[i]=0;
        fri_to_krl.realData[i]=0.0;
    }

    m_toFRI.set(fri_to_krl);
}

void FriRTNetExampleAbstract::stopKrlScript(){

    //Put 3 in $FRI_FRM_INT[1] to trigger fri_stop()
    fri_to_krl.intData[0]=3;
    m_toFRI.set(fri_to_krl);
}


void FriRTNetExampleAbstract::initializeCommand(){
    //Get current joint position and set it as desired position
    if (oport_joint_position.connected()){
        std::vector<double> measured_jointPosition;
        RTT::FlowStatus measured_jointPosition_fs = iport_msr_joint_pos.read(measured_jointPosition);
        if (measured_jointPosition_fs == RTT::NewData){
            std::vector<double> joint_position_command_init;
            joint_position_command_init.assign(7, 0.0);
            for (unsigned int i = 0; i < LWRDOF; i++){
                joint_position_command_init[i] = measured_jointPosition[i];
            }
            oport_joint_position.write(joint_position_command_init);
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

    if (oport_add_joint_trq.connected()){
        //Send 0 joint torque
        std::vector<double> joint_eff_command;
        joint_eff_command.assign(LWRDOF, 0.0);
        oport_add_joint_trq.write(joint_eff_command);
    }
}

std::vector<double> FriRTNetExampleAbstract::getCartPos(){
	geometry_msgs::Pose  msr_cart_pos;
	std::vector <double> msr_cart_pos_vector(3);
	RTT::FlowStatus CartPos_fs = iport_cart_pos.read(msr_cart_pos);
	msr_cart_pos_vector[0] = (double)msr_cart_pos.position.x;
	msr_cart_pos_vector[1] = (double)msr_cart_pos.position.y;
	msr_cart_pos_vector[2] = (double)msr_cart_pos.position.z;
	//if (CartPos_fs == RTT::NewData)
	return msr_cart_pos_vector;
}

std::vector<double> FriRTNetExampleAbstract::getJacobian(){
	KDL::Jacobian  kuka_jacobian_matrix;
	std::vector <double> kuka_jacobian_vector(42);
	Eigen::MatrixXd Jac(6,7);
	RTT::FlowStatus jacobian_fs = jacobianPort.read(kuka_jacobian_matrix);
	Jac.noalias() = kuka_jacobian_matrix.data;
	for (int i=0;i<Jac.rows();i++){
		for(int j=0;j<Jac.cols();j++){
			kuka_jacobian_vector[6*i+j+i] = (double)Jac(i,j);
		}
	}
	return kuka_jacobian_vector;
}

bool FriRTNetExampleAbstract::connectOJointPosition(){
    assert(peer);
    return oport_joint_position.connectTo(peer->getPort("desJntPos"));
}

bool FriRTNetExampleAbstract::connectOJointVelocities(){
    assert(peer);
    return oport_joint_velocities.connectTo(peer->getPort("desJntVel"));
}

bool FriRTNetExampleAbstract::connectOJointTorque(){
    assert(peer);
    return oport_add_joint_trq.connectTo(peer->getPort("desAddJntTrq"));
}

bool FriRTNetExampleAbstract::connectOCartesianPose(){
    assert(peer);
    return oport_cartesian_pose.connectTo(peer->getPort("desCartPos"));
}

void FriRTNetExampleAbstract::sendJointPosition(std::vector<double> &qdes){
    if (oport_joint_position.connected()){
        if(qdes.size() == LWRDOF){
            oport_joint_position.write(qdes);
        }
        else{
            std::cout << "Input wrong qdes vector size, should be 7" << std::endl;
        }
    }
    return;
}

void FriRTNetExampleAbstract::sendJointVelocities(std::vector<double> &qdotdes){
    if (oport_joint_velocities.connected()){
        if(qdotdes.size() == LWRDOF){
            oport_joint_velocities.write(qdotdes);
        }
        else{
            std::cout << "Input wrong qdotdes vector size, should be 7" << std::endl;
        }
    }
    return;
}

void FriRTNetExampleAbstract::sendAddJointTorque(std::vector<double> &tau){
    if (oport_add_joint_trq.connected()){
        if(tau.size() == LWRDOF){
            oport_add_joint_trq.write(tau);
        }
        else{
            std::cout << "Input wrong tau vector size, should be 7" << std::endl;
        }
    }
    return;
}

void FriRTNetExampleAbstract::sendCartesianPose(std::vector<double> &pose){
    if (oport_add_joint_trq.connected()){
        if(pose.size() == 7){
            geometry_msgs::Pose cart_pose;
            cart_pose.position.x = pose[0];
            cart_pose.position.y = pose[1];
            cart_pose.position.z = pose[2];
            cart_pose.orientation.w = pose[3];
            cart_pose.orientation.x = pose[4];
            cart_pose.orientation.y = pose[5];
            cart_pose.orientation.z = pose[6];
            oport_cartesian_pose.write(cart_pose);
        }
        else{
            std::cout << "Input wrong tau vector size, should be 7" << std::endl;
        }
    }
    return;
}
