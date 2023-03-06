#include "Controller.h"

Controller::Controller(Robot* robot, OdometryData* odData)
{
    this->robot = robot;
    this->odData = odData;
    this->Kp = 1;
    this->Ki = 0;
    this->Kd = 0;
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
}

Controller::~Controller()
{
    delete this->robot;
}

Controller::ControllOutput Controller::regulate()
{
    // Calculate regulation error
    Controller::ErrorValue error = Controller::calculateErrors();
    // Check desired heading angle
    double headingAngle = atan2(this->desiredY - this->odData->posY,
                                this->desiredX - this->odData->posX);
//    headingAngle = headingAngle * (180 / PI);

    std::cout << "Heading angle = " << headingAngle << std::endl;

    double rotation = 0.1 * headingAngle;
    double distError = sqrt(error.x*error.x + error.y*error.y);
    double forwardSpeed = distError;
    double radius = 2 * 0.23 * sin(error.theta) / error.theta;
    double arcSpeed = forwardSpeed/radius;

    ControllOutput controllerOutput = {
        forwardSpeed,
        arcSpeed,
        radius,
        rotation
    };

    return controllerOutput;
}

Controller::ErrorValue Controller::calculateErrors()
{
    double eX = abs(this->desiredX - this->odData->posX);
    double eY = abs(this->desiredY - this->odData->posY);
    double eTheta = atan2(this->desiredY - this->odData->posY,
                          this->desiredX - this->odData->posX) - this->odData->rotation * PI / 180;

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


