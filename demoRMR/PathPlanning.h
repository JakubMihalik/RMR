#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#define DEBUG

// Includes
#include "ControlLogic.h"
#include "ObjectDetection.h"
#include <iostream>
#include <fstream>
#include <queue>

class PathPlanning
{
// Constructor and destructor
public:
    PathPlanning(std::ofstream& mapFile, double realWidth, double realHeight, double mapWidth, double mapHeight);
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
    std::ofstream& mapFile;

};

#endif // PATHPLANNING_H
