#ifndef BUGALGORITHM_H
#define BUGALGORITHM_H

#include "Controller.h"
#include "PathPlanning.h"
#include "rplidar.h"

#define BUG_ALG

class BugAlgorithm
{
public:
    BugAlgorithm(Robot* robot, Controller* controller, Point destination, double distanceThreshold, double fovThreshold, double sensorDistanceThreshold);
    ~BugAlgorithm();

/** Public variables **/
public:
    double distanceThreshold;
    double fovThreshold;
    double sensorDistanceThreshold;
    Point finish;
    std::atomic_bool b_followingWall;

/** Private variables **/
private:
    Robot* robot;
    Controller* controller;
    LaserMeasurement laser;
    Point position = {0.0, 0.0};
    Point destination;

/** Public methods **/
public:
    void updatePosition(Point position);
    void updateLidar(LaserMeasurement laser);

/** Private methods **/
public: // TODO: Change to private
    void findObstacle();
    void followObstacle();
};

#endif // BUGALGORITHM_H
