#include "BugAlgorithm.h"

BugAlgorithm::BugAlgorithm(Controller* controller, OdometryData* odData, Robot* robot)
{
    this->robotState = MOVE_TO_GOAL;
    this->controller = controller;
    this->odData = odData;
    this->robot = robot;
    this->start = {odData->posX, odData->posY};
    this->goal = {controller->checkpoints.front().x, controller->checkpoints.front().y};
}

// robot sa toci jak jebnuty, treba nieco upravit pravdepodobne je stale najdeny ten isty point co je tiez pekna picovina

bool BugAlgorithm::obstacleDetection()
{
    double closestDistance = (std::numeric_limits<double>::max)();

    for (int i{0}; i< this->_laserData.numberOfScans; i++)
    {
        double robotAngleRad = DEG2RAD(odData->rotation);
        double lidarAngleRad = DEG2RAD((360.0 - _laserData.Data[i].scanAngle)) - robotAngleRad;
        double lidarDistance = _laserData.Data[i].scanDistance/1000;
        if (abs(lidarAngleRad) < SAFE_ANGLE) {
            if (lidarDistance < this->SAFE_DISTANCE)
            {
                if (lidarDistance < closestDistance)
                {
                    closestDistance = lidarDistance;
                    this->startingFollowDistanceToGoal = distance(controller->checkpoints.front().x, controller->checkpoints.front().y, odData->posX, odData->posY);
                    this->startingFollowPosition = {odData->posX, odData->posY};

                    std::cout<<"Obstacle nearby! Distance: "<< lidarDistance << std::endl;
                    std::cout<<"Obstacle nearby! Angle: "<< lidarAngleRad << std::endl;
                    return true;
                }
            }
        }
    }
    return false;
}

void BugAlgorithm::processRawLidarData(LaserMeasurement laserData)
{
    this->lidarDataXY.clear();
    this->obstacles.clear();

    for (int i {0}; i<laserData.numberOfScans; i++) {
        double robotAngleRad = odData->rotation * PI / 180.0;
        double lidarAngleRad = (360.0 - laserData.Data[i].scanAngle) * PI / 180.0;
        double x = odData->posX + (laserData.Data[i].scanDistance / 1000.0) * cos(lidarAngleRad + robotAngleRad);
        double y = odData->posY + (laserData.Data[i].scanDistance / 1000.0) * sin(lidarAngleRad + robotAngleRad);
        if (laserData.Data[i].scanDistance / 1000.0 <= SAFE_DISTANCE)
        {
            obstacles.push_back({x, y});
        }
        this->lidarDataXY.push_back({x, y});
    }
}

void BugAlgorithm::followObstacle()
{
    Point currentPos = this->start;
    Controller::ErrorValue ev;
    while (distance(currentPos.x, currentPos.y, this->goal.x, this->goal.y) > 0.1)
    {
        if (this->robotState == MOVE_TO_GOAL && !isObstacleInPath())
        {
            currentPos = goal;
            ev = controller->calculateErrors(currentPos, 0, 0);
            controller->regulate(ev);
        }
        else
        {
            if (this->robotState == FOLLOW_OBSTACLE) {
                // Find the first obstacle hit
                isObstacleInPath();
                // Move along the obstacle boundary
                double maxGoalDistanceIncrease = 1; // Set an appropriate threshold based on your environment
                currentPos = findNextObstaclePoint(currentPos, this->goal, this->obstacles, maxGoalDistanceIncrease);
                ev = controller->calculateErrors(currentPos, PI/2 + this->odData->rotation * PI / 180 , SAFE_DISTANCE);
                controller->regulate(ev);
                std::cout<<"Found a point! x: " << currentPos.x << " y: " << currentPos.y << std::endl;
                this->obstacleIndex = (obstacleIndex + 1) % obstacles.size();
                if (!isObstacleInPath())
                {
                    this->robotState = MOVE_TO_GOAL;
                }
            }
        }
    }
}

bool BugAlgorithm::isObstacleInPath()
{
    for (size_t i = 0; i < obstacles.size(); i++) {
        Point p1 = obstacles[i];
        Point p2 = obstacles[(i + 1) % obstacles.size()];
        if (doLineSegmentsIntersect(this->start, this->goal, p1, p2)) {
            this->obstacleIndex = i;
            this->robotState = FOLLOW_OBSTACLE;
            return true;
        }
    }
    return false;
}

bool BugAlgorithm::doLineSegmentsIntersect(Point start, Point goal, Point p2, Point q2) {
    double a1 = goal.y - start.y;
    double b1 = start.x - goal.x;
    double c1 = a1 * start.x + b1 * start.y;

    double a2 = q2.y - p2.y;
    double b2 = p2.x - q2.x;
    double c2 = a2 * p2.x + b2 * p2.y;

    double det = a1 * b2 - a2 * b1;

    if (det == 0) {
        return false; // Line segments are parallel
    } else {
        double x = (b2 * c1 - b1 * c2) / det;
        double y = (a1 * c2 - a2 * c1) / det;
        bool isXWithinSegments = (x >= min(start.x, goal.x) && x <= max(start.x, goal.x) &&
                                     x >= min(p2.x, q2.x) && x <= max(p2.x, q2.x));
        bool isYWithinSegments = (y >= min(start.y, goal.y) && y <= max(start.y, goal.y) &&
                                     y >= min(p2.y, q2.y) && y <= max(p2.y, q2.y));
        return isXWithinSegments && isYWithinSegments;
    }
}

Point BugAlgorithm::findNextObstaclePoint(const Point& currentPos, const Point& goal, const std::vector<Point>& obstacles, double maxGoalDistanceIncrease) {
    Point nextPoint = currentPos;
    double minTotalDistance = (std::numeric_limits<double>::max)();
    double currentDistanceToGoal = this->distance(currentPos, goal);

    for (const Point& obstaclePoint : obstacles) {
        double distanceToObstaclePoint = distance(currentPos, obstaclePoint);
        double distanceToGoal = distance(obstaclePoint, goal);

        // Check if the increase in the distance to the goal is within the allowed threshold
        if (distanceToGoal <= currentDistanceToGoal + maxGoalDistanceIncrease) {
            double totalDistance = distanceToObstaclePoint + distanceToGoal;
            if (totalDistance < minTotalDistance) {
                minTotalDistance = totalDistance;
                nextPoint = obstaclePoint;
            }
        }
    }

    return nextPoint;
}

