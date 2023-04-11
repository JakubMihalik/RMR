#ifndef BUGMOVEMENT_H
#define BUGMOVEMENT_H

#include "RobotLogic.h"
#include "Controller.h"

class BugMovement
{
   public:
       BugMovement(RobotLogic*, Controller*, OdometryData*);
       ~BugMovement();

       // methods
       void initAlgorithm(const std::vector<std::pair<double, double>>& laserData, const std::vector<std::pair<double, double>>& laserDataXY);
       void setRobotState(const std::vector<std::pair<double, double>>& laserData,  const std::vector<std::pair<double, double>>& laserDataXY, float obstacleThreshold, float thresholdAngle);
       int setTurningDirectionBasedOnWallAngle(double wallAngle);
       Controller::ErrorValue followObstacle(const std::vector<std::pair<double, double>>& lidarDataXY);

       // variables and structures

   private:
       RobotLogic* robotLogic;
       Controller* controller;
       OdometryData* odData;
};

#endif // BUGMOVEMENT_H
