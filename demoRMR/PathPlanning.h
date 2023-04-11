#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#define DEBUG

// Includes
#include "Odometry.h"
#include "ObjectDetection.h"
#include <iostream>
#include <fstream>
#include <queue>

class PathPlanning
{
// Constructor and destructor
public:
    PathPlanning(std::ifstream& mapFile, double realWidth, double realHeight, double mapWidth, double mapHeight);
    ~PathPlanning();

// Methods
public:
    std::queue<CheckPoint> planPath(double startX, double startY, double finishX, double finishY);

// Member variables
public:
    double realWidth, realHeight;
    double mapWidth, mapHeihgt;
    double startX, startY;
    double finishX, finishY;
private:
    std::ifstream& mapFile;

};

#endif // PATHPLANNING_H
