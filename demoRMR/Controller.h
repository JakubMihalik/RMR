#ifndef CONTROLLER_H
#define CONTROLLER_H

#pragma once

/** Dependant includes **/
#include "robot.h"
#include "ControlLogic.h"
#include <cmath>

#define deg2rad(d) ((d * 3.1415926536) / 180.0)

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

    typedef struct
    {
        double forwardSpeed;
        double rotationSpeed;
        double reached;
    } ControllerOutput;

/** Public methods **/
public:
    ErrorValue calculateErrors();
    ControllerOutput regulate();

    void setDesiredPosition(double x, double y);
    void setGains(double Kp, double Ki, double Kd);
    void setOffset(double offset);
};

#endif // CONTROLLER_H
