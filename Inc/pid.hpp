#ifndef _PID_HPP_
#define _PID_HPP_

#include<iostream>

namespace CSIR{

/**
 * @brief simple PID implementation, not thread safe.
 * 
 */
class PID{

    private:

    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral_error;


    public:
    
    PID() = default;
    PID(double dt, double max, double min, double Kp, double Kd, double Ki):
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral_error(0)
    {}

    ~PID() = default;


    double calculate(double desire_value, double feedback){
        double error = desire_value - feedback;
        
        //proportional term
        double P = _Kp * error;

        //intergral term
        _integral_error += error * _dt;
        double I = _Ki * _integral_error;

        //derivative term
        double D = -_Kd * (error - _pre_error) / _dt;

        double output = P + I + D;
        if( output > _max) output = _max;
        if( output < _min) output = _min;

        _pre_error = error;

        return output;
    }
    
};


};

#endif
