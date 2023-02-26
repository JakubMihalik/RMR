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
    // Check positive overflow
    if (data->leftWheelTicks > 65200 && robotdata.EncoderLeft < 300)
    {
        // Revolute ocured
        data->leftWheelRevolution++;
    }
    if (data->rightWheelTicks > 65200 && robotdata.EncoderRight < 300)
    {
        // Revolute ocured
        data->rightWheelRevolution++;
    }
    // Check negative overflow
    if (data->leftWheelTicks < 300 && robotdata.EncoderLeft > 65200)
    {
        data->leftWheelRevolution--;
    }
    if (data->rightWheelTicks < 300 && robotdata.EncoderRight > 65200)
    {
        data->rightWheelRevolution--;
    }

    // Update distance of wheels
    data->distLeftWheel += TICK_TO_METER * (robotdata.wheelCurrentLeft - data->leftWheelTicks);
    data->distRightWheel += TICK_TO_METER * (robotdata.wheelCurrentRight - data->rightWheelTicks);

    // Save new wheels encoder values
    data->leftWheelTicks = robotdata.EncoderLeft;
    data->rightWheelTicks = robotdata.EncoderRight;

    // Update rotation
    data->rotation = robotdata.GyroAngle;

    return *data;
}

void ControlLogic::forwardMove(Robot* robot, int speed)
{
    robot->setTranslationSpeed(speed);
}

void ControlLogic::reverseMove(Robot* robot, int speed)
{
    robot->setTranslationSpeed(- speed);
}

void ControlLogic::leftMove(Robot* robot, double speed)
{
    robot->setRotationSpeed(speed);
}

void ControlLogic::rightMove(Robot* robot, double speed)
{
    robot->setRotationSpeed(- speed);
}

void ControlLogic::autonomousRide(Robot* robot, OdometryData data)
{

}
