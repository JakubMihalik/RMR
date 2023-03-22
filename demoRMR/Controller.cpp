#include "Controller.h"

Controller::Controller(Robot* robot, OdometryData* odData)
{
    this->robot = robot;
    this->odData = odData;
    this->Kp = 1;
    this->Ki = 0;
    this->Kd = 0;

    /*CheckPoint p1 = {0, 3};
    CheckPoint p2 = {2.6, 3};
    CheckPoint p3 = {2.6, 0.5};*/
    CheckPoint finish = {4, 0.5};

    /*this->checkpoints.push(p1);
    this->checkpoints.push(p2);
    this->checkpoints.push(p3);*/
    this->checkpoints.push(finish);
}

Controller::Controller(Robot* robot, OdometryData* odData, double desiredX, double desiredY, double Kp, double Ki, double Kd, double offset)
{
    this->robot = robot;
    this->odData = odData;
    this->desiredX = desiredX;
    this->desiredY = desiredY;
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->offset = offset;

//    CheckPoint p1 = {1, 0.1};
    CheckPoint finish = {0, 5};

    this->checkpoints.push(finish);
}

Controller::~Controller()
{
    delete this->robot;
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
//    controllerOutput.forwardSpeed = min(controllerOutput.forwardSpeed, 400);


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

/*Controller::ErrorValue Controller::calculateErrors()
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
}*/

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

/** Getters and Setters  **/
void Controller::setDesiredPosition(double x, double y)
{
    this->desiredX = x;
    this->desiredY = y;
}

void Controller::setGains(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void Controller::setOffset(double offset)
{
    this->offset =  offset;
}


