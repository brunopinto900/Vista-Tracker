#pragma once

class PID
{
public:

    PID(double kp, double ki, double kd);

    double update(double error, double dt);

private:

    double kp_;
    double ki_;
    double kd_;

    double integral_;
    double previous_error_;
};