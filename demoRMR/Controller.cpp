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

    LaserData front = lidar.Data[lidar.numberOfScans - 1];
    LaserData right = lidar.Data[(int)(lidar.numberOfScans * (3.0 / 4.0))];
    LaserData left = lidar.Data[(int)(lidar.numberOfScans * (1.0 / 4.0))];

    const static double desiredDistance = ROBOT_DIAMETER_MM;
    const static int speed = 150;
    const static int radius = ROBOT_RADIUS_MM / 3;
    const static double threshold = 100;

    if (front.scanDistance < desiredDistance + threshold) // Ak je pred nami stena
    {
        /*if (left.scanDistance < desiredDistance) // Ak je stena na lavo
        {
            turnLeft(speed, radius);
        }
        else // Ak je stena na pravo
        {
            turnRight(speed, radius);
        }*/
        if (left.scanDistance < right.scanDistance) // Ak je lava stena blizsie
        {
            turnRight(speed, radius);
        }
        else // Ak je prava stena blizsie
        {
            turnLeft(speed, radius);
        }
    }
    else
    {
        if (left.scanDistance < right.scanDistance) // Ak je stane na lavo
        {
            if (left.scanDistance > desiredDistance + threshold) // Too far from wall
            {
                turnLeft(speed, radius * 2);
            }
            else if (left.scanDistance < desiredDistance) // Too close to wall
            {
                turnRight(speed, radius);
            }
            else // In range
            {
                moveForward(speed);
            }
        }
        else // Ak je stena na pravo
        {
            if (right.scanDistance > desiredDistance + threshold) // Too far from wall
            {
                turnRight(speed, radius * 2);
            }
            else if (right.scanDistance < desiredDistance) // Too close to wall
            {
                turnLeft(speed, radius);
            }
            else // In range
            {
                moveForward(speed);
            }
        }
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
