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

double currentRamp = 1;

Controller::ControllerOutput Controller::regulate()
{
    Controller::ErrorValue ev = Controller::calculateErrors();
    static Controller::ControllerOutput controllerOutput;

    double reqFwdSpeed = 1000 * sqrt(pow(ev.x, 2) + pow(ev.y, 2));
    double reqRotSpeed = 50 * ev.theta;

//    std::cout << "EV.X: " << ev.x << std::endl;
//    std::cout << "EV.Y: " << ev.y << std::endl << std::endl;

    if (abs(ev.x) < 0.01 && abs(ev.y) < 0.01)
    {
        robot->setTranslationSpeed(0);
        return controllerOutput;
    }

    if (reqFwdSpeed > controllerOutput.forwardSpeed) // Pridavame
        controllerOutput.forwardSpeed += 10 * sqrt(pow(ev.x, 2) + pow(ev.y, 2));
    if (controllerOutput.forwardSpeed > reqFwdSpeed) // Spomalujeme
        controllerOutput.forwardSpeed -= 30 * sqrt(pow(ev.x, 2) + pow(ev.y, 2));
    controllerOutput.forwardSpeed = max(min(min(controllerOutput.forwardSpeed, reqFwdSpeed), 400), 0); // A orezeme hranice

    if (reqRotSpeed > controllerOutput.rotationSpeed) // Pridavame
        controllerOutput.rotationSpeed += 0.05 * ev.theta;
    if (controllerOutput.rotationSpeed > reqRotSpeed) // Spomalujeme
        controllerOutput.rotationSpeed -= 0.05 * ev.theta;
    controllerOutput.rotationSpeed = min(controllerOutput.rotationSpeed, reqRotSpeed); // A orezeme hranice
    if (controllerOutput.rotationSpeed > PI / 3)
        controllerOutput.rotationSpeed = PI /3;
    if (controllerOutput.rotationSpeed < - PI / 3)
        controllerOutput.rotationSpeed = - PI / 3;

    double radius = 32768;
    if (abs(controllerOutput.rotationSpeed) > deg2rad(10.0) && abs(controllerOutput.forwardSpeed) > 10)
    {
        radius = controllerOutput.forwardSpeed / controllerOutput.rotationSpeed;
    }

    std::cout << "FW: " << controllerOutput.forwardSpeed << std::endl;
    std::cout << "RS: " << controllerOutput.rotationSpeed << std::endl;
    std::cout << "RA: " << radius << std::endl << std::endl;

    robot->setArcSpeed(controllerOutput.forwardSpeed, radius);

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


