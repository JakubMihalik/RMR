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

OdometryData ControlLogic::readOdometry(TKobukiData robotdata, OdometryData* data)
{
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
    /******** [E] - Detect overflow ********/

    data->lDelta = (65535 * data->leftWheelOverflow) + robotdata.EncoderLeft - data->leftWheelTicks;
    data->rDelta = (65535 * data->rightWheelOverflow) + robotdata.EncoderRight - data->rightWheelTicks;

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
    data->deltaTheta = (data->distRightWheel - data->distLeftWheel) / (2 * 0.23); // 0.23 je rozchod kolies

    // Calculate global position X, Y
    data->posX += data->distance * cos(data->rotation * PI / 180.0);
    data->posY += data->distance * sin(data->rotation * PI / 180.0);

    // Save new wheels encoder values
    data->leftWheelTicks = robotdata.EncoderLeft;
    data->rightWheelTicks = robotdata.EncoderRight;

    // Reset flags
    data->rightWheelOverflow = 0;
    data->leftWheelOverflow = 0;


    return *data;
}
