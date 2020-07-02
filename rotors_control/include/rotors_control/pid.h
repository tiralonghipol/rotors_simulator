#pragma once
class PIDImpl;
class PID
{
public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    PID(double dt, double max, double min, double Kp, double Kd, double Ki, double alpha);

    // Returns the manipulated variable given a setpoint and current process value
    double calculate(double setpoint, double pv);
    void update_gains(double P, double D, double I, double alpha);
    void reset();

    ~PID();

private:
    PIDImpl *pimpl;
};
