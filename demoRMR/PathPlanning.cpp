#include "PathPlanning.h"

PathPlanning::PathPlanning(std::ifstream& mapFile, double realWidth, double realHeight, double mapWidth, double mapHeight) : mapFile(mapFile)
{

}

PathPlanning::~PathPlanning()
{

}

std::queue<CheckPoint> PathPlanning::planPath(double startX, double startY, double finishX, double finishY)
{
    std::queue<CheckPoint> pathPoints;

// Floodfill algorithm
    // Convert finish position to coresponding occupancy map resolution
#ifdef DEBUG
    if (this->mapFile.is_open())
    {
        char c;
        while (this->mapFile.get(c))
        {
            std::cout << c;
        }
    }
#endif


    return pathPoints;
}
