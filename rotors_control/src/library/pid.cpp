#include <iostream>
#include <cmath>
#include "rotors_control/pid.h"

using namespace std;

class PIDImpl
{
public:
    PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki, double alpha);
    ~PIDImpl();
    double calculate(double setpoint, double pv);
    void update_gains(double P, double D, double I, double alpha);
    void reset();

private:
    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _error_prev = 0.0;
    double _integral = 0.0;
    double _alpha = 0.5;
    double _Dout_pre = 0.0;
};

PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki, double alpha)
{
    pimpl = new PIDImpl(dt, max, min, Kp, Kd, Ki, alpha);
}
double PID::calculate(double setpoint, double pv)
{
    return pimpl->calculate(setpoint, pv);
}

void PID::update_gains(double P, double D, double I, double alpha)
{
    return pimpl->update_gains(P, D, I, alpha);
}
void PID::reset()
{
    return pimpl->reset();
}

PID::~PID()
{
    delete pimpl;
}

PIDImpl::PIDImpl(double dt,
                 double max, double min,
                 double Kp, double Kd, double Ki,
                 double alpha) : _dt(dt),
                                 _max(max),
                                 _min(min),
                                 _Kp(Kp),
                                 _Kd(Kd),
                                 _Ki(Ki),
                                 _error_prev(0),
                                 _integral(0),
                                 _alpha(0.0)
{
}

void PIDImpl::update_gains(double P, double D, double I, double alpha)
{
    _Kp = P;
    _Kd = D;
    _Ki = I;
    _alpha = alpha;
}
double PIDImpl::calculate(double setpoint, double pv)
{

    // Error
    double error = setpoint - pv;
    // std::cout << "error = " << error << endl;
    // Proportional
    double Pout = _Kp * error;

    // Integral
    _integral += error * _dt;
    double Iout = _Ki * _integral;
    if (Iout > 0.5 * _max)
        Iout = 0.5 * _max;
    else if (Iout < 0.5 * _min)
        Iout = 0.5 * _min;

    // Derivative
    double derivative = (error - _error_prev) / _dt;
    double Dout_tmp = _Kd * derivative;
    double Dout = _alpha * _Dout_pre + (1 - _alpha) * Dout_tmp;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Check bounds
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    // Swap errors
    _error_prev = error;
    _Dout_pre = Dout;

    return output;
}

void PIDImpl::reset()
{
    _integral = 0.0;
    _error_prev = 0.0;
    _Dout_pre = 0.0;
}

PIDImpl::~PIDImpl()
{
}