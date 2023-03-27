#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

// Depandant includes
#include "ControlLogic.h"
#include "rplidar.h"
#include <iostream>
#include <fstream>
#include <stack>

// Custom defines
#define MAP_SIZE 120

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
    int map2D[MAP_SIZE][MAP_SIZE] = {{0}};
    static constexpr double MAP_RESOLUTION = 0.05;

public:
    DistanceMeasure readLaserData(LaserMeasurement laser);
    void writeLidarMap(std::ofstream& file, OdometryData data, LaserMeasurement laser);
    void avoidObstacles(LaserMeasurement laser, OdometryData robotData, std::stack<CheckPoint>& checkpoints);
    void writeMap2D(std::ofstream& file);
};

#endif // OBJECTDETECTION_H
