#include "ATIcalibration-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <cmath> 
// 0x0042 = code de bias pour le capteur!!
ATIcalibration::ATIcalibration(std::string const& name) : FriRTNetExampleAbstract(name)
{
 this->addPort("ATI_i", iport_ATI_values);
 this->addPort("ATI_calibration_results", oport_calibration_results);
 this->addOperation("setFRIRate", &ATIcalibration::setFRIRate, this, RTT::OwnThread);
 valeurZ.resize(6);
 valeurX.resize(6);
 valeurY.resize(6);
 tf_min.resize(LWRDOF);
 FRIRate=0.02; //20ms
 velocity_limit=0.2; //T1: 250mm/s selon l'end effector, choix arbitraire de 0.15 rad/s
 end_calibration=false;
 t=0;
 position1.resize(LWRDOF);
 position2.resize(LWRDOF);
 position3.resize(LWRDOF);
 JState_init.resize(LWRDOF);
 joints_position_command.resize(LWRDOF);
 joints_position_command_interp.resize(LWRDOF);
 std::cout << "ATIcalibration constructed !" <<std::endl;
}

ATIcalibration::~ATIcalibration(){
}

bool ATIcalibration::configureHook(){

 position1[0]=0;
 position1[1]=0;
 position1[2]=0;
 position1[3]=1.57;
 position1[4]=0;
 position1[5]=-1.57;
 position1[6]=0;

 position2[0]=0;
 position2[1]=0;
 position2[2]=0;
 position2[3]=1.57;
 position2[4]=0;
 position2[5]=0;
 position2[6]=0;

 position3[0]=0;
 position3[1]=0;
 position3[2]=0;
 position3[3]=1.57;
 position3[4]=0;
 position3[5]=0;
 position3[6]=1.57;


 joints_position_command = position1;

  std::cout << "ATIcalibration configured !" <<std::endl;
  return true;
}

bool ATIcalibration::doStart(){
 setControlStrategy(10);
 RTT::FlowStatus joint_state_fs=iport_msr_joint_pos.read(JState_init);
 //if(joint_state_fs == RTT::NewData){
 	for(i=0;i<7;i++){
 		tf_min[i]=(15*std::abs(joints_position_command[i]-JState_init[i])/(8*velocity_limit));
		std::cout << "tf_min["<< i <<"]= " <<tf_min[i]<<" JState_init["<< i <<"]= "<<JState_init[i]<<std::endl;
		std::cout << joints_position_command[i]<< " " <<JState_init[i] <<" "<<std::abs(joints_position_command[i]-JState_init[i])<<std::endl;
 	}
 	tf=tf_min[0];
 	for(i=1;i<7;i++){
		if(tf_min[i]>tf){
			tf=tf_min[i];
		}
 	}
	friStart();
	std::cout << "ATIcalibration started !" <<std::endl;
	std::cout << "tf= " <<tf<<std::endl;
	return true;
// }else
/* {
	std::cout << "Cannot read robot position, fail to start" << std::endl;
        return false;
 }*/
}

void ATIcalibration::updateHook(){
 fri_frm_krl = m_fromFRI.get();
 if(fri_frm_krl.intData[0] == 1){ //command mode
 std::vector<double> JState;
 //requiresControlMode(10);
 //getFRIMode();
 JState.resize(LWRDOF);
 RTT::FlowStatus joint_state_fs =iport_msr_joint_pos.read(JState);
 if(joint_state_fs == RTT::NewData){
 	if(!end_calibration)
 	{
		if(joints_position_command == position1 && t==tf)
 		{
			//on enregistre la valeur des composantes du capteur
			iport_ATI_values.read(valeurZ);
			std::cout<< "valeur lu position 1 (Z) = "<< " Fx= "<< valeurZ[0] <<" Fy= "<< valeurZ[1]<<" Fz= "<< valeurZ[2] << " Tx= "<<valeurZ[3]<<" Ty= "<<valeurZ[4] << " Tz= "<<valeurZ[5] <<std::endl;
			//on change de position de commande
			joints_position_command = position2;
			JState_init=JState;
			t=0;
			for(i=0;i<7;i++){
				tf_min[i]=(15*std::abs(joints_position_command[i]-JState_init[i])/(8*velocity_limit));
			}
			tf=tf_min[0];
		 	for(i=1;i<7;i++){
				if(tf_min[i]>tf){
					tf=tf_min[i];
				}
 			}

 		}else
 		{
			if(joints_position_command == position2 && t == tf)
 			{
				//on enregistre la valeur des composantes du capteur
        			iport_ATI_values.read(valeurX);
				std::cout<< "valeur lu position 2 (X) = "<< " Fx= "<< valeurX[0] <<" Fy= "<< valeurX[1]<<" Fz= "<< valeurX[2] << " Tx= "<<valeurX[3]<<" Ty= "<<valeurX[4] << " Tz= "<<valeurX[5] <<std::endl;
        			//on change de position de commande
        			joints_position_command = position3;
				JState_init=JState;
				t=0;
				for(i=0;i<7;i++){
					tf_min[i]=(15*std::abs(joints_position_command[i]-JState_init[i])/(8*velocity_limit));
				}
				tf=tf_min[0];
				for(i=1;i<7;i++){
					if(tf_min[i]>tf){
						tf=tf_min[i];
					}
 				}

 			}else
			{
				if(joints_position_command == position3 && t==tf)
        			{
                			//on enregistre la valeur des composantes du capteur
                			iport_ATI_values.read(valeurY);
                			std::cout<< "valeur lu position 3 (Y) = "<< " Fx= "<< valeurY[0] <<" Fy= "<< valeurY[1]<<" Fz= "<< valeurY[2] << " Tx= "<<valeurY[3]<<" Ty= "<<valeurY[4] << " Tz= "<<valeurY[5] <<std::endl;
                			//on change de position de commande
					JState_init=JState;
					end_calibration = true;
        			}
			}
 		}
 	}else
 	{
		//calibration terminée
		//changer les attributs de ATISensor qui se débrouillera avec les calculs de rotation etc...
		Eigen::MatrixXd results(3,6);
		results.row(0)=Eigen::VectorXd::Map(&valeurZ[0],valeurZ.size());
		results.row(1)=Eigen::VectorXd::Map(&valeurX[0],valeurX.size());
		results.row(2)=Eigen::VectorXd::Map(&valeurY[0],valeurY.size());
		oport_calibration_results.write(results);
		FriRTNetExampleAbstract::stopHook();
 	}
 	//avant (ici) il faut faire l'interpolation
 	for(i=0;i<7;i++){
 		joints_position_command_interp[i]=JState_init[i]+(joints_position_command[i]-JState_init[i])*(10*pow(t/tf,3)-15*pow(t/tf,4)+6*pow(t/tf,5));
 	}
	
	if(oport_joint_position.connected()){
 		oport_joint_position.write(joints_position_command_interp);
	}
	
 	t+=FRIRate;
 	if(t>tf)
 	{
		t=tf;
 	}
 }
 	/*se stoppe tout seul à la fin stopHook()*/
	//  std::cout << "ATIcalibration executes updateHook !" <<std::endl;
}
}

void ATIcalibration::setFRIRate(double period_ms){
 FRIRate=period_ms;
 std::cout << "FRI period set to "<< FRIRate << " ms" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ATIcalibration)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(ATIcalibration)
