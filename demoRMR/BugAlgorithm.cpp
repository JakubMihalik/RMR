#include "BugAlgorithm.h"
#include "PathPlanning.h"

BugAlgorithm::BugAlgorithm(Robot* robot, Controller* controller, Point destination, double distanceThreshold, double fovThreshold, double sensorDistanceThreshold)
{
    this->robot = robot;
    this->controller = controller;
    this->destination = destination;
    this->finish = destination;
    this->distanceThreshold = distanceThreshold;
    this->fovThreshold = fovThreshold;
    this->sensorDistanceThreshold = sensorDistanceThreshold;

    this->b_followingWall = false;

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
    static double previousDistance = 0;
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
    }
    double distanceToFinish = sqrt(pow(this->finish.x - this->position.x, 2) + pow(this->finish.y - this->position.y, 2)) * 1000.0;
    if (!this->b_followingWall && maxDist != -DBL_MAX && previousDistance < maxDist && maxDist < distanceToFinish)
    {
        // Add that obstacle as target
        double obstacleX = this->position.x + (maxDist - ROBOT_DIAMETER) * cos(DEG2RAD(angle)) / 1000.0;
        double obstacleY = this->position.y - (maxDist - ROBOT_DIAMETER) * sin(DEG2RAD(angle)) / 1000.0;

        this->controller->checkpoints.push({obstacleX, obstacleY});
        this->b_followingWall = true;
        std::cout << "Added point [" << obstacleX << ", " << obstacleY << "]" << std::endl;
    }
    previousDistance = maxDist;
}

void BugAlgorithm::followObstacle()
{

}
