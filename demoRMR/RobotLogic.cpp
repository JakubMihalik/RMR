#include "RobotLogic.h"

RobotLogic::RobotLogic()
{

}

RobotLogic::~RobotLogic()
{

}

void RobotLogic::initControl()
{

}

bool RobotLogic::obstacleInPath(const std::vector<std::pair<double, double>>& lidarData, float thresholdDistance, float thresholdAngle)
{
    for (const auto& point : lidarData) {
        float distance = point.first;
        float angle = point.second;

        if (abs(angle) <= thresholdAngle && distance < thresholdDistance) {
            return true;
        }
    }
    return false;
}


NearestWall RobotLogic::findNearestWall(const std::vector<std::pair<double, double>>& lidarData) {
    float minDistance = std::numeric_limits<float>::max();
    float nearestWallAngle = 0.0;

    for (const auto& point : lidarData) {
        float distance = point.first;
        float angle = point.second;

        if (distance < minDistance) {
            minDistance = distance;
            nearestWallAngle = angle;
        }
    }

    return {minDistance / 1000, nearestWallAngle};
}


bool RobotLogic::isPathToGoalFree(const std::vector<std::pair<double, double>>& laserData, double goalX, double goalY, float obstacleThreshold)
{
    for (const auto& point : laserData)
    {
        double distance = point.first;
        double angle = point.second;

        double pointX = distance * cos(angle);
        double pointY = distance * sin(angle);

        double goalPointDist = sqrt(pow(goalX - pointX, 2) + pow(goalY - pointY, 2));

        if (goalPointDist < obstacleThreshold)
        {
            return false;
        }
    }

    return true;
}


OdometryData RobotLogic::readOdometry(TKobukiData robotdata, OdometryData* data, bool useRotationOdometry)
{
    static double prevRotation = data->initRotation;
    /******** [B] - Detect overflow ********/
    // Left Wheel
    if ((data->leftWheelTicks - robotdata.EncoderLeft) > 30000)
        data->leftWheelOverflow++;
    if ((data->leftWheelTicks - robotdata.EncoderLeft) < -30000)
        data->leftWheelOverflow--;
    // Right Wheel
    if ((data->rightWheelTicks - robotdata.EncoderRight) > 30000)
        data->rightWheelOverflow++;
    if ((data->rightWheelTicks - robotdata.EncoderRight) < -30000)
        data->rightWheelOverflow--;
    data->lDelta = (65535 * data->leftWheelOverflow) + robotdata.EncoderLeft - data->leftWheelTicks;
    data->rDelta = (65535 * data->rightWheelOverflow) + robotdata.EncoderRight - data->rightWheelTicks;
    /******** [E] - Detect overflow ********/

    // Update distance of wheels
    data->distLeftWheel += TICK_TO_METER * data->lDelta;
    data->distRightWheel += TICK_TO_METER * data->rDelta;

    // Update rotation
    double rotation = fmod((robotdata.GyroAngle / 100.0 - data->initRotation), 360.0);
    if (rotation > 180.0)
        rotation -= 360.0;
    else if (rotation < -180.0)
        rotation += 360.0;
    data->rotation = rotation;

    // Calculate total length
    double dLeftDist =  data->lDelta * TICK_TO_METER;
    double dRightDist = data->rDelta * TICK_TO_METER;
    data->distance = (dLeftDist + dRightDist) / 2;
    data->deltaTheta = (data->distRightWheel - data->distLeftWheel) / (2 * WHEEL_BASE_METES);

    if (useRotationOdometry)
    {
        /** Odometry while rotating **/
        double wheelRatio = (WHEEL_BASE_METES * (dRightDist + dLeftDist)) / (2 * (dRightDist - dLeftDist));
        double sinDelta = sin(DEG2RAD(rotation)) - sin(DEG2RAD(prevRotation));
        double cosDelta = cos(DEG2RAD(rotation)) - cos(DEG2RAD(prevRotation));
        data->posX += wheelRatio * sinDelta;
        data->posY -= wheelRatio * cosDelta;
        /** Odometry while rotating **/
    }
    else
    {
        /** Odometry for forward movement **/
        data->posX += data->distance * cos(DEG2RAD(data->rotation));
        data->posY += data->distance * sin(DEG2RAD(data->rotation));
        /** Odometry for forward movement **/
    }

    // Save previous rotation
    prevRotation = rotation;

    // Save new wheels encoder values
    data->leftWheelTicks = robotdata.EncoderLeft;
    data->rightWheelTicks = robotdata.EncoderRight;

    // Reset flags
    data->rightWheelOverflow = 0;
    data->leftWheelOverflow = 0;

    return *data;
}
