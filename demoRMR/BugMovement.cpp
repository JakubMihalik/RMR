#include "BugMovement.h"

BugMovement::BugMovement(RobotLogic *robotLogic, Controller *controller, OdometryData *odData)
{
    this->robotLogic = robotLogic;
    this->controller = controller;
    this->odData = odData;
}

void BugMovement::initAlgorithm(const std::vector<std::pair<double, double>>& laserData, const std::vector<std::pair<double, double>>& laserDataXY)
{
    float obstacleThreshold = 800; // mm
    float thresholdAngle = 30;
    setRobotState(laserData, laserDataXY, obstacleThreshold, thresholdAngle);
    controller->regulate();
}

void BugMovement::setRobotState(const std::vector<std::pair<double, double>>& laserData, const std::vector<std::pair<double, double>>& laserDataXY, float obstacleThreshold, float thresholdAngle)
{
    bool obstacleInPath = robotLogic->obstacleInPath(laserData, obstacleThreshold, thresholdAngle);
    if (robotLogic->robotState == MOVE_TO_GOAL) {
        if (obstacleInPath) {
            robotLogic->robotState = FOLLOW_WALL;
        }
    }
    else if (robotLogic->robotState == FOLLOW_WALL) {
        if (canMoveToGoal()) {
           robotState = MOVE_TO_GOAL;
       } else {
           followObstacle();
       }
        if (!obstacleInPath) {
            robotLogic->robotState = MOVE_TO_GOAL;
        }
        else
        {
            this->followObstacle(laserDataXY);
        }

    }
}

Controller::ErrorValue BugMovement::followObstacle(const std::vector<std::pair<double, double>>& lidarDataXY) {
    // Find the closest obstacle point
    double minDistance = 10000000000;
    std::pair<double, double> closest_obstacle_point;
    for (const auto& point : lidarDataXY) {
        double distanceToObstacle = controller->distance(odData->posX, odData->posY, point.first, point.second);
        if (distanceToObstacle < minDistance) {
            minDistance = distanceToObstacle;
            closest_obstacle_point = point;
        }
    }

    // Compute the angle to the closest obstacle point
    double angleToObstacle = atan2(closest_obstacle_point.second - odData->posY, closest_obstacle_point.first - odData->posX);

    // Turn towards the closest obstacle point
    double desiredAngle = angleToObstacle + PI/2; // Add 90 degrees to move parallel to the obstacle
    controller->error = {minDistance, desiredAngle};
    return {minDistance, desiredAngle};
}

bool BugMovement::canMoveToGoal() {
    double angleToGoal = atan2(controller->checkpoints.top().y - this->odData->posY,
                          controller->checkpoints.top().x - this->odData->posX) - this->odData->rotation * PI / 180;


    for (const auto& point : lidarDataXY) {
        double angle_to_obstacle = atan2(point.second - posY, point.first - posX);
        double angle_diff = abs(angle_to_goal - angle_to_obstacle);

        if (angle_diff < 0.1 && distance(posX, posY, point.first, point.second) < distance(posX, posY, goalX, goalY)) {
            return false;
        }
    }

    return true;
}
