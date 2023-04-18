#include "Controller.h"

Controller::Controller(Robot* robot, OdometryData* odData, double desiredX, double desiredY)
{
    this->robot = robot;
    this->odData = odData;
    this->desiredX = desiredX;
    this->desiredY = desiredY;
    this->fStopLidar = false;
    this->fRotating = false;


//    this->checkpoints.push({0, 3});
//    this->checkpoints.push({1, 3});
//    this->checkpoints.push({1, 1.45});
//    this->checkpoints.push({2.55,1.45});
//    this->checkpoints.push({2.55,0.75});
//    this->checkpoints.push({4.65,0.75});
//    this->checkpoints.push({4.65,1.75});
    this->checkpoints.push({4.5,1.75});
}

Controller::~Controller()
{
    delete this->robot;
    delete this->odData;
}
//Controller::ErrorValue ev = Controller::calculateErrors({this->checkpoints.front().x, this->checkpoints.front().y});


Controller::ControllerOutput Controller::regulate(Controller::ErrorValue ev)
{
    static Controller::ControllerOutput controllerOutput;
    std::cout<<"Error theta: " << ev.theta << " distance: " << ev.distance << std::endl;

    double reqFwdSpeed = 1000 * ev.distance;
    double reqRotSpeed = 10 * ev.theta;

    double rotConst = PI/46;
    double fwdConst = 5;

    // Je v v pozadovanom priestore
    if (abs(ev.distance) < 0.03 && abs(ev.theta) < 0.03)
    {
        robot->setTranslationSpeed(0);
        robot->setRotationSpeed(0);

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

    robot->setArcSpeed(controllerOutput.forwardSpeed, radius);

    return controllerOutput;
}

Controller::ErrorValue Controller::calculateErrors(Point point, double angleBias, double distanceBias)
{
    double eX = abs(point.x - this->odData->posX);
    double eY = abs(point.y - this->odData->posY);

    // Calculate the difference between the current heading and the desired heading
    double eTheta = atan2(point.y - this->odData->posY,
                          point.x - this->odData->posX) - this->odData->rotation * PI / 180 + angleBias;
    if (eTheta > PI) {
        eTheta -= 2*PI;
    } else if (eTheta < -PI) {
        eTheta += 2*PI;
    }
    double distance = sqrt(pow(eX, 2) + pow(eY, 2)) - distanceBias;

    Controller::ErrorValue e = {distance, eTheta};
    return e;
}

void Controller::setCheckpoints(std::queue<Point>& checkpoints)
{
    this->checkpoints = std::move(checkpoints);
}
