#include "BugAlgorithm.h"
#include <chrono>

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
}

void BugAlgorithm::proccessLidar()
{
    auto start = std::chrono::high_resolution_clock::now();

    if (!m_proccessedLidarPoints.empty())
        m_proccessedLidarPoints.clear();

    // Convert lidar data to vector with distance preprocessing
    for (int i{0}; i < m_lidar.numberOfScans; i++)
    {
        if (m_lidar.Data[i].scanDistance > 130 && m_lidar.Data[i].scanDistance < 3000.0)
        {
            int index = i;
            double x = m_position.x + (m_lidar.Data[i].scanDistance - ROBOT_RADIUS_MM) * std::cos(degreesToRadians(m_lidar.Data[i].scanAngle));
            double y = m_position.y + (m_lidar.Data[i].scanDistance - ROBOT_RADIUS_MM) * std::sin(degreesToRadians(m_lidar.Data[i].scanAngle));
            double angle = degreesToRadians(m_lidar.Data[i].scanAngle);
            double distance = m_lidar.Data[i].scanDistance - ROBOT_RADIUS_MM;
            bool isColliding = false; // TODO: Collision detection

            m_proccessedLidarPoints.push_back({index, x, y, angle, distance, isColliding});

            m_lidarFile << x << "," << y << std::endl;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Duration of function: " << duration.count() * 1000.0 << " ms" << std::endl;
}

void BugAlgorithm::updateLidar(LaserMeasurement lidar)
{
    m_lidar = lidar;
}

void BugAlgorithm::updatePosition(Point position)
{
    m_position = position * 1000.0;
}
