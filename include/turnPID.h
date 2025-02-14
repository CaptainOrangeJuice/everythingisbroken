#pragma once
#ifndef turnPID_H
#define turnPID_H

#include "cmath"
#include "vex.h"
#include "motors.h"
#include <iostream>


class turnPID
{

// Try: Increasing kD, make integral = 0 when changes signs and increase end error value

    double position;
    double error;
    double i; // integral
    double d;
    int target;
    double kp = 0.7;
    double ki = 0;
    double kd = 0.15;
    double drive;
    double prev;

public:
    turnPID();
    void reset();
    void tpUpdate();
    void stopTurnPID();
    void runTurnPID(double targetVal/*, vex::motor_group Left, vex::motor_group Right*/);
    // void shake(double seconds);
};

#endif