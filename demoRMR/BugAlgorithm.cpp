#include "BugAlgorithm.h"
#include <chrono>
#include <algorithm>

#define DEBUG

BugAlgorithm::BugAlgorithm(Point destination)
{
    m_destionationPosition = destination;
    m_obstaclePointsFile.open("C:\\RMR_Files\\bugAlg\\obstacle_points.csv");
    m_lidarFile.open("C:\\RMR_Files\\bugAlg\\lidar_points.csv");
}

BugAlgorithm::~BugAlgorithm()
{
    m_proccessedLidarPoints.clear();

    if (m_obstaclePointsFile.is_open())
        m_obstaclePointsFile.close();

    if (m_lidarFile.is_open())
        m_lidarFile.close();
}

void BugAlgorithm::proccess()
{
    proccessLidar();

    // Check if there's obstacle closer then safe distance in front of desired destination
    int index = -1;
    for (LidarPoint p : m_proccessedLidarPoints)
    {
        if (p.distance <= ROBOT_RADIUS_MM)
        {
            index = p.index;
            break;
        }
    }
    if (index != -1)
    {
        LidarPoint p = m_proccessedLidarPoints.at(index);
        std::cout << "D, A = " << p.distance << ", " << p.angle << std::endl;
    }

}

void BugAlgorithm::proccessLidar()
{
    if (!m_proccessedLidarPoints.empty())
        m_proccessedLidarPoints.clear();

    // Convert lidar data to vector with distance preprocessing
    for (int i{0}; i < m_lidar.numberOfScans; i++)
    {
        if (m_lidar.Data[i].scanDistance > 130 && m_lidar.Data[i].scanDistance < 3000.0)
        {
            double robotAngle = m_robotInfo.rotation;
            double rawAngle = 360.0 - m_lidar.Data[i].scanAngle;

            int index = i;
            double angle = degreesToRadians(rawAngle + robotAngle);
            double distance = m_lidar.Data[i].scanDistance - ROBOT_RADIUS_MM;
            double x = m_position.x + distance * std::cos(angle);
            double y = m_position.y + distance * std::sin(angle);
            bool isColliding = false; // TODO: Collision detection

            m_proccessedLidarPoints.push_back({index, x, y, angle, distance, isColliding});

#ifdef DEBUG
            m_lidarFile << x << "," << y << std::endl;
#endif
        }
    }
    // Sort lidar vector based on angle
    std::sort(m_proccessedLidarPoints.begin(), m_proccessedLidarPoints.end(), [](LidarPoint& p1, LidarPoint& p2){return p1.angle < p2.angle;});
}

void BugAlgorithm::updateRobotState(OdometryData robotState)
{
    m_robotInfo = robotState;
    m_position = {robotState.posX * 1000.0, robotState.posY * 1000.0};
}

void BugAlgorithm::updateLidar(LaserMeasurement lidar)
{
    m_lidar = lidar;
}
