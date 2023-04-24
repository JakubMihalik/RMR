#include "Controller.h"

Controller::Controller(Robot* robot, OdometryData* odData, double desiredX, double desiredY)
{
    this->robot = robot;
    this->odData = odData;
    this->desiredX = desiredX;
    this->desiredY = desiredY;
    this->fStopLidar = false;
    this->fRotating = false;

    this->gainForward = 2000;
    this->gainRotation = 10;
    this->rotConst = PI / 32;
    this->fwdConst = 10;
    this->forwardLimit = 1500;
    this->rotationLimit = PI / 2;
    this->accuracy = 0.03;

#ifndef ENABLE_CHECKPOINTS
    this->checkpoints.push({0, 3});
    this->checkpoints.push({2.7, 3});
    this->checkpoints.push({2.7, 0.4});
    this->checkpoints.push({4.75, 0.4});
    this->checkpoints.push({4.75, 1.8});
    std::cout << "Default checkpoints used. No dynamic calculation enabled...\n";
#else
    std::cout << "Calculating checkpoints\n";
#endif
}

Controller::~Controller()
{
    delete this->robot;
    delete this->odData;
}

Controller::ControllerOutput Controller::regulate()
{
    Controller::ErrorValue ev = Controller::calculateErrors();
    static Controller::ControllerOutput controllerOutput;

    double distance = sqrt(pow(ev.x, 2) + pow(ev.y, 2));
    double reqFwdSpeed = this->gainForward * distance;
    double reqRotSpeed = this->gainRotation * ev.theta;

    // Je v v pozadovanom priestore
    if (abs(ev.x) < this->accuracy && abs(ev.y) < this->accuracy)
    {
        robot->setTranslationSpeed(0);

        if (this->checkpoints.size() > 1)
        {
            this->checkpoints.pop();
        }
        if (this->checkpoints.size() <= 1)
        {
            this->gainForward = 1000;
            this->gainRotation = 5;
            this->rotConst = PI / 32;
            this->fwdConst = 5;
            this->forwardLimit = 500;
            this->rotationLimit = PI / 3;
            this->accuracy = 0.015;
        }

        return controllerOutput;
    }

    if (reqFwdSpeed - controllerOutput.forwardSpeed > this->fwdConst)
    {
        controllerOutput.forwardSpeed += this->fwdConst;
    }
    else
    {
        controllerOutput.forwardSpeed = reqFwdSpeed;
    }
    controllerOutput.forwardSpeed = min(controllerOutput.forwardSpeed, this->forwardLimit);


    if (controllerOutput.rotationSpeed - reqRotSpeed > this->rotConst)
    {
        controllerOutput.rotationSpeed -= this->rotConst;
    }
    else if (controllerOutput.rotationSpeed - reqRotSpeed < -this->rotConst)
    {
        controllerOutput.rotationSpeed += this->rotConst;
    }
    else
    {
        controllerOutput.rotationSpeed = reqRotSpeed;
    }
    controllerOutput.rotationSpeed = max(min(controllerOutput.rotationSpeed, this->rotationLimit), -this->rotationLimit);

    double denom = controllerOutput.rotationSpeed != 0 ? controllerOutput.rotationSpeed : 0.1;
    double radius = controllerOutput.forwardSpeed / denom;

    // Set stop lidar flag
    this->fStopLidar = abs(denom) > 1.0; // Toto osetrit nie na radar ale na to co robot robi

    // Set rotation flag
    this->fRotating = abs(controllerOutput.rotationSpeed) >= 0.01;

    robot->setArcSpeed(controllerOutput.forwardSpeed, radius);

    return controllerOutput;
}

Controller::ErrorValue Controller::calculateErrors()
{
    double eX = abs(this->checkpoints.front().x - this->odData->posX);
    double eY = abs(this->checkpoints.front().y - this->odData->posY);

    // Calculate the difference between the current heading and the desired heading
    double eTheta = atan2(this->checkpoints.front().y - this->odData->posY,
                          this->checkpoints.front().x - this->odData->posX) - this->odData->rotation * PI / 180;
    if (eTheta > PI) {
        eTheta -= 2*PI;
    } else if (eTheta < -PI) {
        eTheta += 2*PI;
    }
    Controller::ErrorValue e = {eX, eY, eTheta};
    return e;
}

void Controller::setCheckpoints(std::queue<Point>& checkpoints)
{
    this->checkpoints = std::move(checkpoints);
}
