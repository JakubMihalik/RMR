#ifndef CONTROLLER_H
#define CONTROLLER_H

#pragma once

/** Dependant includes **/
#include "robot.h"
#include "ControlLogic.h"
#include <cmath>

class Controller
{
public:
    Controller(Robot*, OdometryData*);
    Controller(Robot*, OdometryData*, double desiredX, double desiredY, double Kp, double Ki, double Kd, double offset);
    ~Controller();

private:
    double Kp;
    double Ki;
    double Kd;
    Robot* robot;
    OdometryData* odData;
    double desiredX;
    double desiredY;
    double offset;

/** Public variables and structures **/
public:
    typedef struct
    {
        double x;
        double y;
        double theta;
    } ErrorValue;

/** Public methods **/
public:
    ErrorValue calculateErrors();
    void regulate();

    void setDesiredPosition(double x, double y);
    void setGains(double Kp, double Ki, double Kd);
    void setOffset(double offset);
};

#endif // CONTROLLER_H
