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
    std::ofstream m_selectedPointsFile;
    std::vector<LidarPoint> m_proccessedLidarPoints;
    LaserMeasurement m_lidar;
    Point m_position;
    OdometryData m_robotInfo;
    Point m_destionationPosition;
public:
    std::atomic_bool isWallFollowing;

/* Methods */
private:
    void proccessLidar();

public:
    void updateLidar(LaserMeasurement lidar);
    void updateRobotState(OdometryData robotState);
    void proccess(std::vector<Point>& checkpoints);
    LaserMeasurement getLidarData();
};

#endif // BUGALGORITHM_H
