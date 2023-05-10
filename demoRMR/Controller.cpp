#include "Controller.h"
#include "BugAlgorithm.h"

Controller::Controller(Robot* robot, OdometryData* odData, double desiredX, double desiredY)
{
    this->robot = robot;
    this->odData = odData;
    this->desiredX = desiredX;
    this->desiredY = desiredY;
    this->checkpoints.push_back({desiredX, desiredY});
    this->fStopLidar = false;
    this->fRotating = false;
    this->controllerOutput = {0.0, 0.0, 0.0};
}

Controller::~Controller()
{
    delete this->robot;
    delete this->odData;
}

ControllerOutput Controller::regulate()
{
    ErrorValue ev = Controller::calculateErrors();

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
            std::cout << "Checkpoint reached. Poping out..." << std::endl;
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
    this->fStopLidar = abs(controllerOutput.rotationSpeed) >= PI / 12; // Toto osetrit nie na radar ale na to co robot robi

    // Set rotation flag
    this->fRotating = abs(controllerOutput.rotationSpeed) >= PI / 12;

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
//    std::cout << "Going for point: " << checkpoints.back().x << ", " << checkpoints.back().y << std::endl;
    return e;
}

void Controller::setCheckpoints(std::vector<Point>& checkpoints)
{
    this->checkpoints = std::move(checkpoints);
}

void Controller::regulateDynamic(LaserMeasurement lidar)
{
    /** Zaporna hodnota RADIUS toci smerom do lava
     *  Zatacanie do lava (ku stene pre pravy senzor) (-) zaporna hodnota
     *  Zatacanie do prava (od steny pre pravy senzor)(+) kladna hodnota
    **/

    // Find closest point
    LaserData point = {1, 0, DBL_MAX};
    for (int i{0}; i < lidar.numberOfScans; i++)
    {
        if (lidar.Data[i].scanDistance > 130 && lidar.Data[i].scanDistance < 3000)
        {
//            if (lidar.Data[i].scanAngle <= 90 || lidar.Data[i].scanAngle >= 270)
//            {
                if (lidar.Data[i].scanDistance < point.scanDistance)
                {
                    point = lidar.Data[i];
                }
//            }
        }
    }

    if (point.scanDistance != DBL_MAX)
    {
        double angleShift = (-90) + (point.scanDistance - ROBOT_DIAMETER_MM) / 5;
        distance = point.scanDistance;
        angle = point.scanAngle + angleShift;
        angle = angle > 180 ? angle - 360 : angle < -180 ? angle + 360 : angle;

        // Transform angle 90 degree
        double pointX = this->odData->posX + (point.scanDistance / 1000) * std::cos(degreesToRadians(this->odData->rotation + (360 - point.scanAngle)));
        double pointY = this->odData->posY + (point.scanDistance / 1000) * std::sin(degreesToRadians(this->odData->rotation + (360 - point.scanAngle)));

        double reqX = (distance * std::cos(degreesToRadians(this->odData->rotation + (360 - angle)))) / 1000.0;
        double reqY = (distance * std::sin(degreesToRadians(this->odData->rotation + (360 - angle)))) / 1000.0;

//        double reqX = (pointX * std::cos(PI / 2) + pointY * std::sin(PI / 2));
//        double reqY = (-pointX * std::sin(PI / 2) + pointY * std::cos(PI / 2));

        // Add new checkpoint
        if (checkpoints.size() < 2)
        {
            checkpoints.push_back({reqX, reqY});
            std::cout << "Found point: " << point.scanAngle << "d " << point.scanDistance << "mm\n";
            std::cout << "X, Y: " << pointX << "m " << pointY << "m\n";
            std::cout << "Added [" << reqX << ", " << reqY << "]\n" << std::endl;
        }
        else if (checkpoints.size() > 1)
        {
            checkpoints.pop_back();
            checkpoints.push_back({reqX, reqY});
        }
    }
}

void Controller::regulateWallFollow(double reqX, double reqY)
{
    ErrorValue error = calculateErrors();
    // Check if position is achieved
    if (abs(error.x) < 0.1 && abs(error.y) < 0.1)
    {
        if (checkpoints.size() > 1)
        {
            checkpoints.pop_back();
        }
    }
    // Align angle

    if (abs(error.theta) > degreesToRadians(5.0))
    {
        robot->setRotationSpeed(error.theta);
    }
    else
    {
        robot->setTranslationSpeed(150);
    }
}

void Controller::turnLeft(int speed, int radius)
{
    robot->setArcSpeed(speed, radius);
}

void Controller::turnRight(int speed, int radius)
{
    robot->setArcSpeed(speed, -radius);
}

void Controller::moveForward(int speed)
{
    robot->setArcSpeed(speed, 0);
}
