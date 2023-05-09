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
    m_selectedPointsFile.open("C:\\RMR_Files\\bugAlg\\selected_points.csv");

    isWallFollowing = false;
}

BugAlgorithm::~BugAlgorithm()
{
    m_proccessedLidarPoints.clear();

    if (m_obstaclePointsFile.is_open())
        m_obstaclePointsFile.close();
    std::cout << "Points file closed" << std::endl;

    if (m_lidarFile.is_open())
        m_lidarFile.close();
    std::cout << "Lidar file closed" << std::endl;

    if (m_selectedPointsFile.is_open())
        m_selectedPointsFile.close();
    std::cout << "Selected points file closed" << std::endl;
}

void BugAlgorithm::proccess(std::vector<Point>& checkpoints)
{
    proccessLidar();

    // Check if there's obstacle closer then safe distance in front of desired destination
    double range = 90.0;
    LidarPoint maxDistance = {-1, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, false};
    bool danger = false;
    int dangerCounts = 0;

    if (!this->isWallFollowing)
    {
        for (LidarPoint p : m_proccessedLidarPoints)
        {
            // Check if point is from range and in the other way
            bool b_firstQuadrant = p.angle < degreesToRadians(range);
            bool b_fourthQuadrant = p.angle < degreesToRadians(360.0) && p.angle > degreesToRadians(360.0 - range);

            // If its in defined range then check if distance is less then safe distance
            if ((b_firstQuadrant || b_fourthQuadrant) && p.distance - ROBOT_RADIUS_M <= 0.3)
            {
                // Todo avoidance couse we are heading toward obstacle
                danger = true;
                dangerCounts++;
                if (p.distance > maxDistance.distance)
                {
                    maxDistance = p;
                }
            }
        }
        // Simple filter
        if (danger && dangerCounts > 2)
        {
//            checkpoints.push_back({maxDistance.x, maxDistance.y});
//            m_selectedPointsFile << checkpoints.back().x << "," << checkpoints.back().y << std::endl;
            this->isWallFollowing = true;
        }
    }
    else
    {
        // Check if we can go to the finish
        double headingAngle = atan2(m_destionationPosition.y - m_position.y, m_destionationPosition.x - m_position.x);
        double headingAngleDeg = radiansToDegrees(headingAngle);

        double robotRotation = m_robotInfo.rotation;
        double rotation = headingAngleDeg + robotRotation;

        int freeWaysCount = 0;

        for (int i{0}; i < m_lidar.numberOfScans; i++)
        {
            bool b_upperBound = m_lidar.Data[i].scanAngle < rotation + 5.0;
            bool b_lowerBound = m_lidar.Data[i].scanAngle > rotation - 5.0;
            bool b_distanceBound = m_lidar.Data[i].scanDistance > 1500;

            if (b_upperBound && b_lowerBound && b_distanceBound)
            {
                freeWaysCount++;
            }
        }
        if (freeWaysCount > 3)
        {
//            this->isWallFollowing = false;
        }
    }
}

void BugAlgorithm::proccessLidar()
{
    if (!m_proccessedLidarPoints.empty())
        m_proccessedLidarPoints.clear();

    // Convert lidar data to vector
    for (int i{0}; i < m_lidar.numberOfScans; i++)
    {
        if (m_lidar.Data[i].scanDistance > 130 && m_lidar.Data[i].scanDistance < 3000.0)
        {
            int index = i;
            double robotAngle = degreesToRadians(m_robotInfo.rotation);
            double angle = degreesToRadians(360.0 - m_lidar.Data[i].scanAngle);
            double distance = m_lidar.Data[i].scanDistance / 1000.0; // Distance is in meters
            double x = m_position.x + distance * std::cos(angle + robotAngle);
            double y = m_position.y + distance * std::sin(angle + robotAngle);

            bool isColliding = false; // TODO: Collision detection

            m_lidarFile << x << "," << y << "," << angle + robotAngle << std::endl;
            m_proccessedLidarPoints.push_back({index, x, y, angle, distance, isColliding});
        }
    }
    // Sort lidar vector based on angle
    std::sort(m_proccessedLidarPoints.begin(), m_proccessedLidarPoints.end(), [](LidarPoint& p1, LidarPoint& p2){return p1.angle < p2.angle;});
}

void BugAlgorithm::updateRobotState(OdometryData robotState)
{
    m_robotInfo = robotState;
    m_position = {robotState.posX, robotState.posY};
}

void BugAlgorithm::updateLidar(LaserMeasurement lidar)
{
    m_lidar = lidar;
}

LaserMeasurement BugAlgorithm::getLidarData()
{
    return m_lidar;
}
