#ifndef CONTROLLER_H
#define CONTROLLER_H

#pragma once

/** Dependant includes **/
#include "robot.h"
#include "Odometry.h"
#include <cmath>
#include <queue>

#define deg2rad(d) ((d * 3.1415926536) / 180.0)


class Controller
{
public:
    Controller(Robot*, OdometryData*, double desiredX, double desiredY);
    ~Controller();

private:
    Robot* robot;
    OdometryData* odData;
    double desiredX;
    double desiredY;

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

    std::queue<CheckPoint> checkpoints;
    std::atomic<bool> fStopLidar;
    std::atomic<bool> fRotating;

/** Public methods **/
public:
    ErrorValue calculateErrors();
    ControllerOutput regulate();
};

#endif // CONTROLLER_H
