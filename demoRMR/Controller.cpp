#include "Controller.h"
#include "RobotLogic.h"
#include "ObjectDetection.h"

Controller::Controller(RobotLogic *robotLogic, Robot* robot, OdometryData* odData, double desiredX, double desiredY)
{
    this->robotLogic = robotLogic;
    this->robot = robot;
    this->odData = odData;
    this->desiredX = desiredX;
    this->desiredY = desiredY;
    this->fStopLidar = false;
    this->fRotating = false;
    this->checkpoints.push({4.5, 1.75});
//    this->checkpoints.push({4.5, 0.5});
//    this->checkpoints.push({4, 0.5});
//    this->checkpoints.push({2.6, 0.5});
//    this->checkpoints.push({2.6, 3});
//    this->checkpoints.push({0, 3});
}

Controller::~Controller()
{
    delete this->robotLogic;
    delete this->robot;
    delete this->odData;
}


Controller::ControllerOutput Controller::regulate()
{
    Controller::ErrorValue ev = Controller::calculateErrors();
    static Controller::ControllerOutput controllerOutput;

    double reqFwdSpeed = 1000 * ev.distance;
    double reqRotSpeed = 10 * ev.theta;
    double minRadius = 0.1;
    double rotConst = PI/46;
    double fwdConst = 5;
    double speedReductionFactor = 1.0;
    double safeDistance = 0.1;
    // Je v v pozadovanom priestore
    if (ev.distance < 0.03)
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
    controllerOutput.forwardSpeed = min(controllerOutput.forwardSpeed, 500);


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

    controllerOutput.rotationSpeed = max(min(controllerOutput.rotationSpeed, PI / 3), -PI / 3);

    double denom = controllerOutput.rotationSpeed != 0 ? controllerOutput.rotationSpeed : 0.1;
    double radius = controllerOutput.forwardSpeed / denom;

    // Set stop lidar flag
    this->fStopLidar = abs(denom) > 1.0;

    // Set rotation flag
    this->fRotating = abs(controllerOutput.rotationSpeed) >= 0.01;


    if (robotLogic->robotState == FOLLOW_WALL) {
       controllerOutput.forwardSpeed = 100;
    }

    robot->setArcSpeed(controllerOutput.forwardSpeed, radius);

    return controllerOutput;
}

Controller::ErrorValue Controller::calculateErrors()
{
    double distance = this->distance(this->checkpoints.top().x, this->checkpoints.top().y, this->odData->posX,  this->odData->posY);
    // Calculate the difference between the current heading and the desired heading
    double eTheta = atan2(this->checkpoints.top().y - this->odData->posY,
                          this->checkpoints.top().x - this->odData->posX) - this->odData->rotation * PI / 180;

    if (robotLogic->robotState == FOLLOW_WALL) {
        eTheta = this->error.theta;
        distance = this->error.distance;
    }

    if (eTheta > PI) {
        eTheta -= 2*PI;
    } else if (eTheta < -PI) {
        eTheta += 2*PI;
    }

    Controller::ErrorValue e = {distance, eTheta};
    return e;
}



double Controller::distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}
