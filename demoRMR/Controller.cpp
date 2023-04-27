#include "Controller.h"
#include "BugAlgorithm.h"

Controller::Controller(Robot* robot, OdometryData* odData, double desiredX, double desiredY)
{
    this->robot = robot;
    this->odData = odData;
    this->desiredX = desiredX;
    this->desiredY = desiredY;
    this->fStopLidar = false;
    this->fRotating = false;
}

Controller::~Controller()
{
    delete this->robot;
    delete this->odData;
}

ControllerOutput Controller::regulate()
{
    ErrorValue ev = Controller::calculateErrors();
    static ControllerOutput controllerOutput;

    double distance = sqrt(pow(ev.x, 2) + pow(ev.y, 2));
    double reqFwdSpeed = 1000 * distance;
    double reqRotSpeed = 5 * ev.theta;

    double rotConst = PI / 32;
    double fwdConst = 5;


    // Je v v pozadovanom priestore
    if (abs(ev.x) < 0.03 && abs(ev.y) < 0.03)
    {
        robot->setTranslationSpeed(0);

        if (this->checkpoints.size() > 1)
        {
            this->checkpoints.pop_back();
        }
        else
        {
            this->b_finishReached = true;
            std::cout << "Finished" << std::endl;
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
    controllerOutput.forwardSpeed = min(controllerOutput.forwardSpeed, 750);


    if (controllerOutput.rotationSpeed - reqRotSpeed > rotConst)
    {
        controllerOutput.rotationSpeed -= rotConst;
    }
    else if (controllerOutput.rotationSpeed - reqRotSpeed < -rotConst)
    {
        controllerOutput.rotationSpeed += rotConst;
    }
    else
    {
        controllerOutput.rotationSpeed = reqRotSpeed;
    }
    controllerOutput.rotationSpeed = max(min(controllerOutput.rotationSpeed, PI / 2), -PI / 2);

    double denom = controllerOutput.rotationSpeed != 0 ? controllerOutput.rotationSpeed : 0.1;
    double radius = controllerOutput.forwardSpeed / denom;

    // Set stop lidar flag
    this->fStopLidar = abs(denom) > 1.0; // Toto osetrit nie na radar ale na to co robot robi

    // Set rotation flag
    this->fRotating = abs(controllerOutput.rotationSpeed) >= PI / 6;

    robot->setArcSpeed(controllerOutput.forwardSpeed, radius);

    return controllerOutput;
}

ErrorValue Controller::calculateErrors()
{
    double eX = abs(this->checkpoints.back().x - this->odData->posX);
    double eY = abs(this->checkpoints.back().y - this->odData->posY);

    // Calculate the difference between the current heading and the desired heading
    double eTheta = atan2(this->checkpoints.back().y - this->odData->posY,
                          this->checkpoints.back().x - this->odData->posX) - this->odData->rotation * PI / 180;
    if (eTheta > PI) {
        eTheta -= 2*PI;
    } else if (eTheta < -PI) {
        eTheta += 2*PI;
    }
    ErrorValue e = {eX, eY, eTheta};
    return e;
}

void Controller::setCheckpoints(std::vector<Point>& checkpoints)
{
    this->checkpoints = std::move(checkpoints);
}
