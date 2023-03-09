#include "RobotRamp.h"

RobotRamp::RobotRamp()
{
    this->currentSpeed = 0.0;
    this->desiredSpeed = 0.0;
}

double RobotRamp::getCurrentSpeed()
{
    return this->currentSpeed;
}

double RobotRamp::getDesiredSpeed()
{
    return this->desiredSpeed;
}

void RobotRamp::setCurrentSpeed(double speed)
{
    this->currentSpeed = speed;
}

double RobotRamp::setDesiredSpeed(double speed)
{
    this->desiredSpeed = speed;

    // Ak aktualna rychlost je nizsia ako zelena -> zvysujeme
    if (this->desiredSpeed > this->currentSpeed)
    {
        this->currentSpeed += RAMP_INCREMENT_VALUE;
    }

    // Ak je aktualna rychlost vyssia ako zelana -> znizujeme
    else if (this->currentSpeed > this->desiredSpeed)
    {
        this->currentSpeed -= RAMP_DECREMENT_VALUE;
    }

    // Ak je prekorceny maximalny limit rychlosti oreze sa
    else if (this->currentSpeed > MAX_SPEED)
    {
        this->currentSpeed = MAX_SPEED;
    }

    // A to iste pre opacny smer
    else if (this->currentSpeed < -MAX_SPEED)
    {
        this->currentSpeed = -MAX_SPEED;
    }

    // Ak ziadna podmienka neplati (nemalo by nikdy nastat ale pre istotu)
    else
    {
        this->currentSpeed = this->desiredSpeed;
    }

    return this->currentSpeed;
}
