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
    this->b_prepareForFollow = false;

    // Add destination to stack
    this->controller->checkpoints.push_back(this->destination);
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
    double maxDist = -DBL_MAX;
    double angle = 0.0;
    bool b_finishReachable = false;


    double distanceToFinish = sqrt(pow(this->finish.x - this->position.x, 2) + pow(this->finish.y - this->position.y, 2)) * 1000.0;

    // Loop over each laser measurement
    for (int i{0}; i < this->laser.numberOfScans; i++)
    {
        double headingAngle = atan2((this->finish.y - this->position.y), (this->finish.x - this->position.x));
        bool b_validLidarReading = this->laser.Data[i].scanDistance <= this->sensorDistanceThreshold && this->laser.Data[i].scanDistance > 130;
        bool b_inFrontOfFinish = isWithinRange(this->laser.Data[i].scanAngle, rad2degree(headingAngle), 5.0);

        if (b_validLidarReading && b_inFrontOfFinish && this->laser.Data[i].scanDistance > maxDist)
        {
            maxDist = this->laser.Data[i].scanDistance;
            angle = this->laser.Data[i].scanAngle;
        }

        if (isWithinRange(b_validLidarReading && this->laser.Data[i].scanAngle, headingAngle, 2.0) && this->laser.Data[i].scanDistance >= distanceToFinish)
        {
            b_finishReachable = true;
        }
    }

    if (b_finishReachable)
    {
        this->controller->checkpoints.clear();
        this->controller->checkpoints.push_back(this->finish);
    }
    else
    {
        // Calculte obstacle positions
        if (!this->b_prepareForFollow && !this->b_followingWall && maxDist != -DBL_MAX && previousDistance < maxDist && maxDist < distanceToFinish)
        {
            // Add that obstacle as target
            double obstacleX = this->position.x + (maxDist - ROBOT_RADIUS * 1000) * cos(DEG2RAD(angle)) / 1000.0;
            double obstacleY = this->position.y + (maxDist - ROBOT_RADIUS * 1000) * sin(DEG2RAD(angle)) / 1000.0;

            this->controller->checkpoints.push_back({obstacleX, obstacleY});
            this->b_prepareForFollow = true;
            std::cout << "Added point [" << obstacleX << ", " << obstacleY << "]" << std::endl;
            std::cout << "Max dist: " << maxDist << " Angle: " << angle << std::endl;
        }
        previousDistance = maxDist;

        // If robot is prepared to follow wall start following it
        if (this->b_followingWall && !this->b_prepareForFollow)
        {
            followObstacle();
        }
    }
}

void BugAlgorithm::followObstacle()
{
    // We need to align with wall and generate required checkpoints

}

bool BugAlgorithm::isWithinRange(double measured, double reference, double rangeDeg)
{
    double rangeRad = rangeDeg * PI / 180.0;
    double phi360 = fmod(reference, 360.0);
    double theta360 = fmod(measured, 360.0);
    double diff = fabs(theta360 - phi360);
    if (diff > 180.0) {
        diff = 360.0 - diff;
    }
    double diffRad = diff * PI / 180.0;
    return diffRad <= rangeRad;
}

double BugAlgorithm::rad2degree(double rad)
{
    return rad * 180.0 / PI;
}
