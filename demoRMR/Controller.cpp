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

    if (ev.theta < -0.01 || ev.theta > 0.01) {
        controllerOutput.rotationSpeed = 10 * ev.theta;
        if (controllerOutput.rotationSpeed > PI/3.0)
        {
            controllerOutput.rotationSpeed = PI/3.0;
        }
//        robot->setRotationSpeed(controllerOutput.rotationSpeed);
    }
    if (ev.x < -0.1 || ev.x > 0.1 || ev.y < -0.1 || ev.y > 0.1) {
        controllerOutput.forwardSpeed = 100 * sqrt(ev.x*ev.x + ev.y*ev.y);
        if (controllerOutput.forwardSpeed > 400)
        {
            controllerOutput.forwardSpeed = 400;
        }

//        robot->setTranslationSpeed(controllerOutput.forwardSpeed);
    }  else {
        controllerOutput.reached = 1;
        robot->setRotationSpeed(controllerOutput.rotationSpeed);
        robot->setRotationSpeed(controllerOutput.forwardSpeed);
    }

    if (controllerOutput.rotationSpeed > 0.1) {
        robot->setArcSpeed(controllerOutput.forwardSpeed, controllerOutput.forwardSpeed / controllerOutput.rotationSpeed);
    } else if (controllerOutput.forwardSpeed > 0.1) {
        robot->setArcSpeed(controllerOutput.forwardSpeed, 32768);
    }



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


