#ifndef BUGALGORITHM_H
#define BUGALGORITHM_H

#include <fstream>
#include <vector>
#include "types.h"
#include "rplidar.h"

class BugAlgorithm
{
public:
    BugAlgorithm(Point destination);
    ~BugAlgorithm();

/* Variables */
private:
    std::ofstream m_obstaclePointsFile;
    std::ofstream m_lidarFile;
    std::vector<LidarPoint> m_proccessedLidarPoints;
    LaserMeasurement m_lidar;
    Point m_position;
    Point m_destionationPosition;

/* Methods */
private:
    void proccessLidar();

public:
    void updateLidar(LaserMeasurement lidar);
    void updatePosition(Point position);
    void proccess();
};

#endif // BUGALGORITHM_H