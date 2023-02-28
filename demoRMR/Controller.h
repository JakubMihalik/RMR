#ifndef CONTROLLER_H
#define CONTROLLER_H

#pragma once

/** Dependant includes **/
#include "robot.h"
#include "ControlLogic.h"

class Controller
{
public:
    Controller(Robot*, OdometryData*);
    Controller(Robot*, OdometryData*, double, double, double);
    ~Controller();

private:
    double Kp;
    double Ki;
    double Kd;
    Robot* robot;
    OdometryData* odData;

/** Public variables and structures **/
public:
    typedef struct
    {
        double x;
        double y;
    } ErrorValue;

/** Public methods **/
public:
    ErrorValue calculateErrors();
    void setRegulation();
};

#endif // CONTROLLER_H
