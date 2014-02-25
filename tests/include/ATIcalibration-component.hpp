#ifndef OROCOS_ATICALIBRATION_COMPONENT_HPP
#define OROCOS_ATICALIBRATION_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include "friRTNetExampleAbstract.hpp"
#include <Eigen/Dense>
class ATIcalibration : public FriRTNetExampleAbstract{
  public:
    ATIcalibration(std::string const& name);
    ~ATIcalibration();
    bool configureHook();
    bool doStart();
    void updateHook();
    void setFRIRate(double period_ms);

    std::vector<double> JState_init;//msr_joint_positions
    bool end_calibration;
    std::vector<double> joints_position_command;
    std::vector<double> joints_position_command_interp;
   //Positions de calibration à vérifier notamment au niveau des axes (Xcapteur = -Xpoignet, Ycapteur=-Ypoignet, Zcapteur=Zpoignet)
    std::vector<double> position1;
    std::vector<double> position2;
    std::vector<double> position3;

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
    RTT::OutputPort< Eigen::Matrix<double,3,6> > oport_calibration_results;
    RTT::OutputPort< bool > oport_bias_order;

};
#endif
