#include "ATIcalibration-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

ATIcalibration::ATIcalibration(std::string const& name) : FriExampleAbstract(name)
{
 this->addPort("ATI_i", iport_ATI_values);
 this->addPort("ATI_calibration_results", oport_calibration_results);
 this->addOperation("setFRIRate", &ATIcalibration::setFRIRate, this, RTT::OwnThread);
  std::cout << "ATIcalibration constructed !" <<std::endl;
}

ATIcalibration::~ATIcalibration(){
}

bool ATIcalibration::configureHook(){
 FRIRate=0.02; //20ms
 joints_position_command = position1;
 velocity_limit=0.15; //T1: 250mm/s selon l'end effector, choix arbitraire de 0.15 rad/s

 position1.positions[0]=0;
 position1.positions[1]=0;
 position1.positions[2]=0;
 position1.positions[3]=1.57;
 position1.positions[4]=0;
 position1.positions[5]=1.57;
 position1.positions[6]=0;

 position2.positions[0]=0;
 position2.positions[1]=0;
 position2.positions[2]=0;
 position2.positions[3]=1.57;
 position2.positions[4]=0;
 position2.positions[5]=0;
 position2.positions[6]=0;

 position3.positions[0]=0;
 position3.positions[1]=0;
 position3.positions[2]=0;
 position3.positions[3]=1.57;
 position3.positions[4]=0;
 position3.positions[5]=0;
 position3.positions[6]=1.57;

 end_calibration=false;
 iport_joint_state.read(JState_init);
 for(i=0;i<7;i++){
 	tf_min[i]=(15*abs(joints_position_command.positions[i]-JState_init.position[i])/(8*velocity_limit));
 }
 tf=tf_min[0];
 for(i=1;i<7;i++){
	if(tf_min[i]>tf){
		tf=tf_min[i];
	}
 }
 t=0;
  std::cout << "ATIcalibration configured !" <<std::endl;
  return true;
}

bool ATIcalibration::doStart(){
  std::cout << "ATIcalibration started !" <<std::endl;
  return true;
}

void ATIcalibration::updateHook(){
 sensor_msgs::JointState JState;

 iport_joint_state.read(JState);

 if(!end_calibration)
 {
 	if(joints_position_command.positions == position1.positions && JState.position == position1.positions)
 	{
		//on enregistre la valeur des composantes du capteur
		iport_ATI_values.read(valeurZ);
		std::cout<< "valeur lu position 1 (Z) = "<< " Fx= "<< valeurZ[0] <<" Fy= "<< valeurZ[1]<<" Fz= "<< valeurZ[2] << " Tx= "<<valeurZ[3]<<" Ty= "<<valeurZ[4] << " Tz= "<<valeurZ[5] <<std::endl;
		//on change de position de commande
		joints_position_command = position2;
		JState_init=JState;
		t=0;
		for(i=0;i<7;i++){
			tf_min[i]=(15*abs(joints_position_command.positions[i]-JState_init.position[i])/(8*velocity_limit));
		}
		tf=tf_min[0];
		 for(i=1;i<7;i++){
			if(tf_min[i]>tf){
				tf=tf_min[i];
			}
 		}

 	}else
 	{
 		if(joints_position_command.positions == position2.positions && JState.position == position2.positions)
 		{
			//on enregistre la valeur des composantes du capteur
        		iport_ATI_values.read(valeurX);
			std::cout<< "valeur lu position 2 (X) = "<< " Fx= "<< valeurX[0] <<" Fy= "<< valeurX[1]<<" Fz= "<< valeurX[2] << " Tx= "<<valeurX[3]<<" Ty= "<<valeurX[4] << " Tz= "<<valeurX[5] <<std::endl;
        		//on change de position de commande
        		joints_position_command = position3;
			JState_init=JState;
			t=0;
			for(i=0;i<7;i++){
				tf_min[i]=(15*abs(joints_position_command.positions[i]-JState_init.position[i])/(8*velocity_limit));
			}
			tf=tf_min[0];
			for(i=1;i<7;i++){
				if(tf_min[i]>tf){
					tf=tf_min[i];
				}
 			}

 		}else
		{
			if(joints_position_command.positions == position3.positions && JState.position == position3.positions)
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
	FriExampleAbstract::stopHook();
 }
 //avant (ici) il faut faire l'interpolation
 for(i=0;i<7;i++){
 	joints_position_command_interp.positions[i]=JState_init.position[i]+(joints_position_command.positions[i]-JState_init.position[i])*(10*pow(t/tf,3)-15*pow(t/tf,4)+6*pow(t/tf,5));
 }
 oport_joint_position.write(joints_position_command_interp);
 t+=FRIRate;
 if(t>tf)
 {
	t=tf;
 }
 /*se stoppe tout seul à la fin stopHook()*/
  std::cout << "ATIcalibration executes updateHook !" <<std::endl;
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
