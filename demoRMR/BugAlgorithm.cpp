#include "BugAlgorithm.h"

BugAlgorithm::BugAlgorithm(Robot* robot, Controller* controller, Point destination, double distanceThreshold, double fovThreshold, double sensorDistanceThreshold)
{
    this->robot = robot;
    this->controller = controller;
    this->destination = destination;
    this->distanceThreshold = distanceThreshold;
    this->fovThreshold = fovThreshold;
    this->sensorDistanceThreshold = sensorDistanceThreshold;

    this->b_isFollowingObstacle = false;

    // Add destination to queue
    this->controller->checkpoints.push(this->destination);
}

BugAlgorithm::~BugAlgorithm()
{
    delete this->robot;
    delete this->controller;
}

void BugAlgorithm::updatePosition(Point position)
{
    this->position = position;
}

void BugAlgorithm::updateLidar(LaserMeasurement laser)
{
    this->laser = laser;
}

void BugAlgorithm::findObstacle()
{
    // Maximal distance available
    double maxDist = -DBL_MAX;
    double angle = 0.0;
    // Loop over each laser measurement
    for (int i{0}; i < this->laser.numberOfScans; i++)
    {
        double cosAngle = cos(DEG2RAD(this->laser.Data[i].scanAngle));
        if (cosAngle >= cos(DEG2RAD(this->fovThreshold)))
        {
            if (this->laser.Data[i].scanDistance <= this->sensorDistanceThreshold &&
                this->laser.Data[i].scanDistance > ROBOT_RADIUS &&
                this->laser.Data[i].scanDistance > maxDist)
            {
                maxDist = this->laser.Data[i].scanDistance;
                angle = this->laser.Data[i].scanAngle;
            }
        }
        if (maxDist != -DBL_MAX && maxDist);
    }
}
