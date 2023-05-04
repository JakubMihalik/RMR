#include "BugAlgorithm.h"
#include <chrono>
#include <algorithm>
#include <vector>

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

void BugAlgorithm::proccess(std::vector<Point>& checkpoints)
{
    proccessLidar();

    // Check if there's obstacle closer then safe distance in front of desired destination
    double range = 90.0;
    int index = -1;
    std::vector<LidarPoint> viewPoints;
    for (LidarPoint p : m_proccessedLidarPoints)
    {
        // Check if point is from range (0-45) and in the other way (360 - 315)
        bool b_firstQuadrant = p.angle < degreesToRadians(range);
        bool b_fourthQuadrant = p.angle < degreesToRadians(360.0) && p.angle > degreesToRadians(360.0 - range);
        // If its in defined range then check if distance is less then safe distance
        if ((b_firstQuadrant || b_fourthQuadrant) && p.distance <= ROBOT_DIAMETER_MM)
        {
            // Todo avoidance couse we are heading toward obstacle
            m_lidarFile << "X:" << p.x << "\tY:" << p.y << "\tAngle:" << radiansToDegrees(p.angle) << std::endl;
            viewPoints.push_back(p);
        }
    }
    // Find largest distance in view angle
    if (!viewPoints.empty())
    {
        LidarPoint p = *std::max_element(viewPoints.begin(), viewPoints.end(), [](const LidarPoint& p1, const LidarPoint& p2){return p1.distance < p2.distance;});
        // Push new chekpoint
        checkpoints.push_back({p.x / 1000.0, p.y / 1000.0});
        std::cout << "Pushed: " << checkpoints.back().x << ", " << checkpoints.back().y << std::endl;
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
            double rawAngle = 360.0 - m_lidar.Data[i].scanAngle;

            int index = i;
            double angle = degreesToRadians(rawAngle);
            double distance = m_lidar.Data[i].scanDistance - ROBOT_RADIUS_MM;
            double x = m_position.x + distance * std::cos(angle);
            double y = m_position.y + distance * std::sin(angle);
            bool isColliding = false; // TODO: Collision detection

            m_proccessedLidarPoints.push_back({index, x, y, angle, distance, isColliding});
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
