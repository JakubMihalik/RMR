#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

// Depandant includes
#include "ControlLogic.h"
#include "rplidar.h"
#include <iostream>
#include <fstream>
#include <stack>

typedef struct
{
    double angle;
    double distance;
} DistanceMeasure;

class ObjectDetection
{
public:
    ObjectDetection();
    ~ObjectDetection();

public:
    DistanceMeasure readLaserData(LaserMeasurement laser);
    void writeLidarMap(std::ofstream& file, OdometryData data, LaserMeasurement laser);
    void avoidObstacles(LaserMeasurement laser, OdometryData robotData, std::stack<CheckPoint>& checkpoints);
};

#endif // OBJECTDETECTION_H
