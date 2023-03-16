#include "Controller.h"

Controller::Controller(Robot* robot, OdometryData* odData)
{
    this->robot = robot;
    this->odData = odData;
    this->Kp = 1;
    this->Ki = 0;
    this->Kd = 0;

    CheckPoint p1 = {0, 3};
    CheckPoint p2 = {2.6, 3};
    CheckPoint p3 = {2.6, 0.5};
    CheckPoint finish = {4, 0.5};

    this->checkpoints.push(p1);
    this->checkpoints.push(p2);
    this->checkpoints.push(p3);
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

    CheckPoint p1 = {0, 3};
    CheckPoint p2 = {2.6, 3};
    CheckPoint p3 = {2.6, 0.5};
    CheckPoint finish = {4, 0.5};

    this->checkpoints.push(p1);
    this->checkpoints.push(p2);
    this->checkpoints.push(p3);
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

    double reqFwdSpeed = 50000 * sqrt(pow(ev.x, 2) + pow(ev.y, 2));

    if (abs(ev.x) < 0.03 && abs(ev.y) < 0.03)
    {
        robot->setTranslationSpeed(0);

        if (this->checkpoints.size() > 1)
        {
            this->checkpoints.pop();
        }

        return controllerOutput;
    }

    if (reqFwdSpeed > controllerOutput.forwardSpeed) { // Pridavame
        controllerOutput.forwardSpeed += 10 * sqrt(pow(ev.x, 2) + pow(ev.y, 2));
    }
    if (controllerOutput.forwardSpeed > reqFwdSpeed) // Spomalujeme
        controllerOutput.forwardSpeed -= 10 * sqrt(pow(ev.x, 2) + pow(ev.y, 2));
    controllerOutput.forwardSpeed = max(min(min(controllerOutput.forwardSpeed, reqFwdSpeed), 600), 0); // A orezeme hranice

    controllerOutput.rotationSpeed = ev.theta * 3;

    double radius = 32768;
    double denom = controllerOutput.rotationSpeed != 0 ? controllerOutput.rotationSpeed : 0.000001;
    radius = controllerOutput.forwardSpeed / denom;

    std::cout << "FW: " << controllerOutput.forwardSpeed << std::endl;
    std::cout << "RS: " << controllerOutput.rotationSpeed << std::endl;
    std::cout << "RA: " << radius << std::endl << std::endl;
    std::cout << "Error theta: " << ev.theta << std::endl << std::endl;
    std::cout << "Error x: " << ev.x << std::endl << std::endl;

    if (abs(ev.theta) < PI - 0.1) {
        robot->setArcSpeed(controllerOutput.forwardSpeed, radius);
    } else {
       robot->setRotationSpeed(controllerOutput.rotationSpeed);
    }

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


