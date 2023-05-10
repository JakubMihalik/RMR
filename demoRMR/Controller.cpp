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
        if (   (lidar.Data[i].scanDistance > 130 && lidar.Data[i].scanDistance < 3000)
            && (lidar.Data[i].scanDistance < 640 || lidar.Data[i].scanDistance > 700))
        {
            if (lidar.Data[i].scanDistance < point.scanDistance)
            {
                point = lidar.Data[i];
//                point.scanAngle = 360.0 - point.scanAngle; // Lidar je opacne tocivy ako robot
            }
        }
    }

    if (point.scanDistance != DBL_MAX)
    {
        std::cout << "Found point: " << point.scanAngle << std::endl;
        std::cout << "With distance: " << point.scanDistance << std::endl;

        // Vypocitame poziciu najblizsieho bodu [mm]
        double wallX = odData->posX + (point.scanDistance - ROBOT_DIAMETER_MM) * std::cos(degreesToRadians(odData->rotation + point.scanAngle));
        double wallY = odData->posY + (point.scanDistance - ROBOT_DIAMETER_MM) * std::sin(degreesToRadians(odData->rotation + point.scanAngle));

        // Vypocitame vektor k najblizsiemu bodu [mm]
        Point vector = {wallX - odData->posX,
                        wallY - odData->posY};

        // Otocime vektor o 90 stupnov [mm]
        Point rotatedVector = {vector.x * std::cos(PI/2) - vector.y * std::sin(PI/2),
                               vector.x * std::cos(PI/2) + vector.y * std::cos(PI/2)};

        // Vypocitame bod na sledovanie [mm]
        double pointX = odData->posX + rotatedVector.x;
        double pointY = odData->posY + rotatedVector.y;

        // Vypocitame uhol a vzdialenost bodu kvoli vykreslovaniu
        m_angle = radiansToDegrees(atan2(odData->posY - pointY, odData->posX - pointX));
        m_distance = std::sqrt(std::pow(odData->posX - pointX, 2) + std::pow(odData->posY - pointY, 2));

        // A prevedieme na metre
        pointX /= 1000.0;
        pointY /= 1000.0;

        // Regulate
        regulateWallFollow(pointX, pointY);
    }
}

void Controller::regulateWallFollow(double reqX, double reqY)
{
    // Vypocitame chyby
    ErrorValue ev;
    double eX = abs(reqX - this->odData->posX);
    double eY = abs(reqY - this->odData->posY);

    // Calculate the difference between the current heading and the desired heading
    double eTheta = atan2(reqY - this->odData->posY,
                          reqX - this->odData->posX) - this->odData->rotation * PI / 180;
    if (eTheta > PI) {
        eTheta -= 2*PI;
    } else if (eTheta < -PI) {
        eTheta += 2*PI;
    }
    ev = {eX, eY, eTheta};

    // Parametre regulacie
    double distance = sqrt(pow(ev.x, 2) + pow(ev.y, 2));
    double reqFwdSpeed = 500 * distance;
    double reqRotSpeed = 5 * ev.theta;

    double rotConst = PI / 32;
    double fwdConst = 5;


    // Je v v pozadovanom priestore
//    if (abs(ev.x) < 0.03 && abs(ev.y) < 0.03)
//    {
//        robot->setTranslationSpeed(0);
//        this->controllerOutput.forwardSpeed = 0;
//        this->controllerOutput.rotationSpeed = 0;
//        return;
//    }

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
    if(abs(controllerOutput.rotationSpeed) >= PI / 12)
    {
        this->fRotating = true;
        this->fStopLidar = true;
    }
    else
    {
        this->fRotating = false;
        this->fStopLidar = false;
    }

//    if (reqRotSpeed / 5 > PI / 2)
//    {
//        robot->setRotationSpeed(PI / 6);
//    }
//    else
//    {
        robot->setArcSpeed(controllerOutput.forwardSpeed, max(min(radius, INT_MAX), -INT_MAX));
//    }

    // Vypis
    std::cout << "Request: [" << reqX << ", " << reqY << "]" << std::endl;
    std::cout << "Forward speed: " << controllerOutput.forwardSpeed << std::endl;
    std::cout << "Radius: " << radius << std::endl;
    std::cout << "Request rotation: " << reqRotSpeed / 5 << std::endl;
    // Koniec
    std::cout << std::endl;
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
