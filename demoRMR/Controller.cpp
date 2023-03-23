#include "Controller.h"

Controller::Controller(Robot* robot, OdometryData* odData, double desiredX, double desiredY)
{
    this->robot = robot;
    this->odData = odData;
    this->desiredX = desiredX;
    this->desiredY = desiredY;

    CheckPoint finish = {desiredX, desiredY};
    this->checkpoints.push(finish);
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
    double reqFwdSpeed = 500 * distance;
    double reqRotSpeed = 10 * ev.theta;

    double rotConst = PI/46;
    double fwdConst = 5;


    // Je v v pozadovanom priestore
    if (abs(ev.x) < 0.03 && abs(ev.y) < 0.03)
    {
        robot->setTranslationSpeed(0);

        if (this->checkpoints.size() > 1)
        {
            this->checkpoints.pop();
        }

        return controllerOutput;
    }

    if (reqFwdSpeed - controllerOutput.forwardSpeed > fwdConst)
    {
        controllerOutput.forwardSpeed += fwdConst;
    }
    else
    {
        controllerOutput.forwardSpeed = reqFwdSpeed;
    }
    controllerOutput.forwardSpeed = min(controllerOutput.forwardSpeed, 400);


    if (controllerOutput.rotationSpeed - reqRotSpeed > rotConst)
    {
        controllerOutput.rotationSpeed -= rotConst;
    }
    else if (reqFwdSpeed - controllerOutput.forwardSpeed > rotConst)
    {
        controllerOutput.rotationSpeed += rotConst;
    }
    else {
        controllerOutput.rotationSpeed = reqRotSpeed;
    }
    controllerOutput.rotationSpeed = max(min(controllerOutput.rotationSpeed, PI / 3), - PI / 3);

    // toto trha robot ak je tam len x!
    double denom = controllerOutput.rotationSpeed != 0 ? controllerOutput.rotationSpeed : 0.000001;
    double radius = controllerOutput.forwardSpeed / denom;

    if (abs(ev.theta) < PI/5) {
        robot->setArcSpeed(controllerOutput.forwardSpeed, radius);
    } else {
       robot->setRotationSpeed(controllerOutput.rotationSpeed);
    }

    return controllerOutput;
}

Controller::ErrorValue Controller::calculateErrors()
{
    double eX = abs(this->checkpoints.top().x - this->odData->posX);
    double eY = abs(this->checkpoints.top().y - this->odData->posY);

    // Calculate the difference between the current heading and the desired heading
    double eTheta = atan2(this->checkpoints.top().y - this->odData->posY,
                          this->checkpoints.top().x - this->odData->posX) - this->odData->rotation * PI / 180;
    if (eTheta > PI) {
        eTheta -= 2*PI;
    } else if (eTheta < -PI) {
        eTheta += 2*PI;
    }
    Controller::ErrorValue e = {eX, eY, eTheta};
    return e;
}
