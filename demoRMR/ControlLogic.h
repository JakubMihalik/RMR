#ifndef CONTROLLOGIC_H
#define CONTROLLOGIC_H

/**
 * Custom class header that contains
 * whole control logic of robot
**/

/**
 * Dependant includes
**/
#include "robot.h"
#include <cmath>

/**
 * Global variables
**/
#define TICK_TO_METER 0.000085292090497737556558

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
    double forwardSpeed;
} OdometryData;

class ControlLogic
{
public:
    ControlLogic();
    ~ControlLogic();

    void initControl();

    OdometryData readOdometry(TKobukiData robotdata, OdometryData* data);

    void forwardMove(Robot* robot, int speed);
    void reverseMove(Robot* robot, int speed);
    void leftMove(Robot* robot, double speed);
    void rightMove(Robot* robot, double speed);

    void navigation(Robot* robot);
    void localisation(Robot* robot);
    void autonomousRide(Robot* robot, OdometryData data);
};

#endif // CONTROLLOGIC_H
