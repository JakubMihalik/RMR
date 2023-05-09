#ifndef CONTROLLER_H
#define CONTROLLER_H

#pragma once

/** Dependant includes **/
#include "robot.h"
#include "Odometry.h"
#include <cmath>
#include <vector>
#include "types.h"

#define deg2rad(d) ((d * 3.1415926536) / 180.0)

//#define ENABLE_CHECKPOINTS

typedef struct
{
    double forwardSpeed;
    double rotationSpeed;
    double reached;
} ControllerOutput;

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
    LaserMeasurement laser;

public:
    std::vector<Point> checkpoints;
    std::atomic<bool> fStopLidar;
    std::atomic<bool> fRotating;
    bool b_finishReached = false;
    ControllerOutput controllerOutput;

/** Public methods **/
public:
    ErrorValue calculateErrors();
    ControllerOutput regulate();
    void regulateDynamic(LaserMeasurement lidar);
    void setCheckpoints(std::vector<Point>& checkpoints);
    void followWall();
    void updateLidarData(LaserMeasurement laser);
    void turnLeft(int speed, int radius);
    void turnRight(int speed, int radius);
    void moveForward(int speed);

};

#endif // CONTROLLER_H
