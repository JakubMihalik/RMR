    #include "ControlLogic.h"

ControlLogic::ControlLogic()
{

}

ControlLogic::~ControlLogic()
{

}

void ControlLogic::initControl()
{

}

OdometryData ControlLogic::readOdometry(TKobukiData robotdata, OdometryData* data, bool useRotationOdometry)
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
    double denom = (2 * (dRightDist - dLeftDist));
    if (useRotationOdometry && denom !=0)
    {
        /** Odometry while rotating **/
        double wheelRatio = (WHEEL_BASE_METES * (dRightDist + dLeftDist)) / denom;
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
