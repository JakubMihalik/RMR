#ifndef ODOMETRY_H
#define ODOMETRY_H

/**
 * Custom class header that contains
 * whole control logic of robot
**/

/**
 * Dependant includes
**/
#include <cmath>
#include "CKobuki.h"
/**
 * Global variables
**/
#define TICK_TO_METER 0.000085292090497737556558
#define WHEEL_BASE_METES 0.23
#define WHEEL_BASE_MILIMETERS 230.0
#define DEG2RAD(d) (d * 3.1415926536 / 180.0)

/**
 * Structure that holds all data
 * from odometry read
**/
typedef struct {
    unsigned short leftWheelTicks;
    unsigned short rightWheelTicks;
    double distRightWheel;
    double distLeftWheel;
    double rotation;
    double posX;
    double posY;
    double distance;
    double deltaTheta;
    int leftWheelOverflow;
    int rightWheelOverflow;
    int rDelta;
    int lDelta;
    double initRotation;
} OdometryData;

/**
 * Checkpoint structure structure
**/
typedef struct {
    double x;
    double y;
} CheckPoint;

class Odometry
{
public:
    Odometry();
    ~Odometry();

    void initControl();

    OdometryData readOdometry(TKobukiData robotdata, OdometryData* data, bool useRotationOdometry = false);
};

#endif // ODOMETRY_H
