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

void Controller::regulate()
{
    // Calculate regulation error
    Controller::ErrorValue error = Controller::calculateErrors();
    // Check desired heading angle
    double headingAngle = atan2(this->desiredY - this->odData->posY,
                                this->desiredX - this->odData->posX);
    std::cout << "Heading angle = " << headingAngle * (180 / PI) << std::endl;
    // Check current angle

    // If heading angle is greater then specified offset start rotating toward desired angle

    // Else move forward
}

Controller::ErrorValue Controller::calculateErrors()
{
    double eX = this->desiredX - this->odData->posX;
    double eY = this->desiredY - this->odData->posY;
    double eTheta = atan2(eY, eX) - this->odData->rotation;

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
