#ifndef ROBOTLOGIC_H
#define ROBOTLOGIC_H

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
 * @brief The RobotState enum
 */
enum RobotState {
    MOVE_TO_GOAL,
    FOLLOW_WALL
};

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

typedef struct
{
   float distance;
   float angle;
} NearestWall;

typedef struct
{
   float x;
   float y;
} NearestWallXY;

class RobotLogic
{
public:
    RobotLogic();
    ~RobotLogic();
    bool obstacleInPath(const std::vector<std::pair<double, double>>& lidarData, float thresholdDistance, float thresholdAngle);
    NearestWall findNearestWall(const std::vector<std::pair<double, double>>& lidarData);
    NearestWallXY findNearestWallXY(const std::vector<std::pair<double, double>>& lidarDataXY);

    bool RobotLogic::isPathToGoalFree(const std::vector<std::pair<double, double>>& laserData, double goalX, double goalY, float obstacleThreshold);
    void initControl();
    OdometryData readOdometry(TKobukiData robotdata, OdometryData* data, bool useRotationOdometry = false);
    RobotState robotState = MOVE_TO_GOAL;
};

#endif // ROBOTLOGIC_H
