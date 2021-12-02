#ifndef _PARAMETERS_HPP_
#define _PARAMETERS_HPP_

#define INITIAL_ANGLE_STATE {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI, M_PI_4}

#include<math.h>

namespace CSIR{

namespace {

const int DOF=7;

const double Kp = 0.0;
const double Ki = 0.0;
const double Kd = 0.0;
const double pid_max = 0.0;
const double pid_min = 0.0;

const double time_update_direvalue = 20;
const double time_control_interval = 1;

const int control_update_max=300;
const int control_update_min=50;

const double max_q[7]={2.6,1.6,2.6,-0.1,2.6,3.5,2.6};
const double min_q[7]={-2.6,-1.6,-2.6,-3.0,-2.6,-0.1,-2.6};
const double max_q_d[7]={2.1,2.17,2.1,2.1,2.6,2.6,2.6};
const double max_q_dd[7]={14.9,7.3, 9.9,12.4,14.9,19.9,19.9};
const double max_q_ddd[7]={7000,3700,4500,5800,7000,9000,9000};
const double max_q_dddd[7]={0};

const int udp_port = 2233;

const char* robot_ip = "172.16.0.2";

};

};
#endif