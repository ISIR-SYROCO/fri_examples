#ifndef OROCOS_ATICALIBRATION_COMPONENT_HPP
#define OROCOS_ATICALIBRATION_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include "friExampleAbstract.hpp"
#include <Eigen/Dense>
class ATIcalibration : public FriExampleAbstract{
  public:
    ATIcalibration(std::string const& name);
    ~ATIcalibration();
    bool configureHook();
    bool doStart();
    void updateHook();
    void setFRIRate(double period_ms);

    sensor_msgs::JointState JState_init;
    bool end_calibration;
    motion_control_msgs::JointPositions joints_position_command;
    motion_control_msgs::JointPositions joints_position_command_interp;
   //Positions de calibration à vérifier notamment au niveau des axes (Xcapteur = -Xpoignet, Ycapteur=-Ypoignet, Zcapteur=Zpoignet)
    motion_control_msgs::JointPositions position1;
    motion_control_msgs::JointPositions position2;
    motion_control_msgs::JointPositions position3;

    double FRIRate;
    double velocity_limit;
    double tf;
    double t;
    std::vector<double> valeurZ;
    std::vector<double> valeurX;
    std::vector<double> valeurY;

    std::vector<double> tf_min;

    int i;

    RTT::InputPort< std::vector<double> > iport_ATI_values;
    RTT::OutputPort< Eigen::Matrix<double,6,3> > oport_calibration_results;
};
#endif
