#ifndef BUGALGORITHM_H
#define BUGALGORITHM_H

#include "Controller.h"
#include "PathPlanning.h"
#include "rplidar.h"

class BugAlgorithm
{
public:
    BugAlgorithm(double distanceThreshold, double fovThreshold, double sensorDistanceThreshold);
    ~BugAlgorithm();

/** Public variables **/
public:
    double distanceThreshold;
    double fovThreshold;
    double sensorDistanceThreshold;
    Point finish;

/** Public methods **/
public:
    void followWall(LaserMeasurement laser, ControllerOutput& controller);
};

#endif // BUGALGORITHM_H
