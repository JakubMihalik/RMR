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
    /******** Detect overflow ********/
    // Left Wheel
    if (data->leftWheelTicks > 65200 && robotdata.EncoderLeft < 300)
        data->leftWheelOverflow++;
    if (data->leftWheelTicks < 300 && robotdata.EncoderLeft > 65200)
        data->leftWheelOverflow--;
    // Right Wheel
    if (data->rightWheelTicks > 65200 && robotdata.EncoderRight < 300)
        data->rightWheelOverflow++;
    if (data->rightWheelTicks < 300 && robotdata.EncoderRight > 65200)
        data->rightWheelOverflow--;
    // TODO: Vyriesit pretecenie gyroskopu

    data->lDelta = (65535 * data->leftWheelOverflow) + robotdata.EncoderLeft - data->leftWheelTicks;
    data->rDelta = (65535 * data->rightWheelOverflow) + robotdata.EncoderRight - data->rightWheelTicks;

    // Update distance of wheels
    data->distLeftWheel += TICK_TO_METER * data->lDelta;
    data->distRightWheel += TICK_TO_METER * data->rDelta;

    // Update rotation
    data->rotation = robotdata.GyroAngle / 100.0;

    // Calculate total length
    double dLeftDist =  data->lDelta * TICK_TO_METER;
    double dRightDist = data->rDelta * TICK_TO_METER;
    data->distance = (dLeftDist + dRightDist) / 2;
    data->deltaTheta = (data->distRightWheel - data->distLeftWheel) / (2*0.23);

    // Calculate global position X, Y
    data->posX += data->distance * cos(data->rotation * PI / 180.0) * 100.0;
    data->posY += data->distance * sin(data->rotation * PI / 180.0) * 100.0;
//    data->rotation += data->deltaTheta;

    // Save new wheels encoder values
    data->leftWheelTicks = robotdata.EncoderLeft;
    data->rightWheelTicks = robotdata.EncoderRight;

    // Reset flags
    data->rightWheelOverflow = 0;
    data->leftWheelOverflow = 0;


    return *data;
}

void ControlLogic::forwardMove(Robot* robot, int speed)
{
    robot->setTranslationSpeed(speed);
}

void ControlLogic::reverseMove(Robot* robot, int speed)
{
    robot->setTranslationSpeed(-speed);
}

void ControlLogic::leftMove(Robot* robot, double speed)
{
    robot->setRotationSpeed(speed);
}

void ControlLogic::rightMove(Robot* robot, double speed)
{
    robot->setRotationSpeed(-speed);
}

void ControlLogic::autonomousRide(Robot* robot, OdometryData data)
{
    if (data.posX > -2.5)
    {
        reverseMove(robot, 100);
        return;
    }
    if (data.rotation < 90)
    {
        leftMove(robot, PI / 4);
        return;
    }
    if (data.posY < 333)
    {
        forwardMove(robot, 250);
        return;
    }
}
