#ifndef CONTROLLER_H
#define CONTROLLER_H

#pragma once

/** Dependant includes **/
#include "robot.h"
#include "RobotLogic.h"
#include <cmath>
#include <queue>
#include <stack>

#define deg2rad(d) ((d * 3.1415926536) / 180.0)


class Controller
{
public:
    Controller(RobotLogic*, Robot*, OdometryData*, double desiredX, double desiredY);
    ~Controller();

private:
    RobotLogic* robotLogic;
    Robot* robot;
    OdometryData* odData;
    LaserMeasurement* laserData;
    double desiredX;
    double desiredY;

/** Public variables and structures **/
public:
    typedef struct
    {
        double distance;
        double theta;
    } ErrorValue;

    typedef struct
    {
        double forwardSpeed;
        double rotationSpeed;
        double reached;
    } ControllerOutput;

    std::stack<CheckPoint> checkpoints;
    std::atomic<bool> fStopLidar;
    std::atomic<bool> fRotating;
    std::atomic<bool> closeToWall;

    NearestWall nearestWall;
    ErrorValue error;

/** Public methods **/
public:
    ErrorValue calculateErrors();
    ControllerOutput regulate();
    double distance(double x1, double y1, double x2, double y2);
};

#endif // CONTROLLER_H
