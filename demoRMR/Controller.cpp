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
    // Calculate regulation error
    Controller::ErrorValue ev = Controller::calculateErrors();
    Controller::ControllerOutput controllerOutput;

    // Check desired heading angle
    double headingAngle = atan2(this->desiredY - this->odData->posY,
                                this->desiredX - this->odData->posX);
//    headingAngle = headingAngle * (180 / PI);
    std::cout << "Heading angle = " << headingAngle << std::endl;
    controllerOutput.reached = 0;

    if (abs(ev.theta) > 1*PI/180) {
        controllerOutput.rotationSpeed = 200 * ev.theta;
        if (controllerOutput.rotationSpeed > PI/3.0)
        {
            controllerOutput.rotationSpeed = PI/3.0;
        }
    } else {
        controllerOutput.rotationSpeed = 32576;
    }
    if (abs(ev.x) > 0.01 || abs(ev.y) > 0.01) {
//        if (controllerOutput.forwardSpeed > sqrt(ev.x*ev.x + ev.y*ev.y)/1000) {
//            controllerOutput.forwardSpeed -= 40;
//        } else {
//            controllerOutput.forwardSpeed += 20;
//        }
        controllerOutput.forwardSpeed += 400; // currentRamp * sqrt(ev.x*ev.x + ev.y*ev.y);
//        if (controllerOutput.forwardSpeed > 400)
//        {
//            controllerOutput.forwardSpeed = 400;
//        }
    }  else {
        controllerOutput.reached = 1;
        robot->setRotationSpeed(0);
        robot->setTranslationSpeed(0);
    }

    std::cout << "FWS = " << controllerOutput.forwardSpeed << std::endl;
    std::cout << "RTS = " << controllerOutput.rotationSpeed << std::endl;

    robot->setArcSpeed(controllerOutput.forwardSpeed, controllerOutput.forwardSpeed / controllerOutput.rotationSpeed);



//    double rotation = 0.1 * headingAngle;
//    double distError = sqrt(error.x*error.x + error.y*error.y);
//    double forwardSpeed = distError;
//    double radius = 2 * 0.23 * sin(error.theta) / error.theta;
//    double arcSpeed = forwardSpeed/radius;

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


