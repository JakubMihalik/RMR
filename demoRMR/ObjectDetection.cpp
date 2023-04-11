#include "ObjectDetection.h"

#define DEBUG

ObjectDetection::ObjectDetection()
{
    // Todo: Constructor
}

ObjectDetection::~ObjectDetection()
{
    // Todo: Destructor
}


std::vector<std::pair<double, double>> ObjectDetection:: readLaserData(LaserMeasurement laser)
{
    std::vector<std::pair<double, double>> lidarData;
    for (int i = 0; i < laser.numberOfScans; i++) {
        lidarData.emplace_back(laser.Data[i].scanDistance, laser.Data[i].scanAngle);
    }

    return lidarData;
}

std::vector<std::pair<double, double>> ObjectDetection:: readLaserDataXY(LaserMeasurement laser, OdometryData data)
{
    std::vector<std::pair<double, double>> lidarDataXY;
    for (int i = 0; i < laser.numberOfScans; i++) {
        double robotAngleRad = data.rotation * PI / 180.0;
        double lidarAngleRad = (360.0 - laser.Data[i].scanAngle) * PI / 180.0;
        double x = data.posX + (laser.Data[i].scanDistance / 1000.0) * cos(lidarAngleRad + robotAngleRad);
        double y = data.posY + (laser.Data[i].scanDistance / 1000.0) * sin(lidarAngleRad + robotAngleRad);
        lidarDataXY.emplace_back(x, y);
    }

    return lidarDataXY;
}

void ObjectDetection::writeLidarMap(std::ofstream& file, OdometryData data, LaserMeasurement laser)
{
    for (int i{0}; i < laser.numberOfScans; i++)
    {
        double robotAngleRad = data.rotation * PI / 180.0;
        double lidarAngleRad = (360.0 - laser.Data[i].scanAngle) * PI / 180.0;
        double x = data.posX + (laser.Data[i].scanDistance / 1000.0) * cos(lidarAngleRad + robotAngleRad);
        double y = data.posY + (laser.Data[i].scanDistance / 1000.0) * sin(lidarAngleRad + robotAngleRad);

        file << x << "," << y << "\n";

        // Binary map
        if (laser.Data[i].scanDistance < 3000.0 /*&& laser.Data[i].scanDistance > 130*/ && laser.Data[i].scanDistance > 640.0)
        {
            int mapX = (MAP_SIZE / 2 - 1) + std::round(x / MAP_RESOLUTION);
            int mapY = (MAP_SIZE / 2 - 1) - std::round(y / MAP_RESOLUTION);
            this->map2D[mapY][mapX] = 1;
        }
    }
}

void ObjectDetection::writeMap2D(std::ofstream& file)
{
    for (int y{0}; y < MAP_SIZE; y++)
    {
        for (int x{0}; x < MAP_SIZE; x++)
        {
            file << (this->map2D[y][x] != 0 ? "X" : " ");
        }
        file << "\n";
    }
    std::cout << "Map written\n";
}

void ObjectDetection::avoidObstacles(LaserMeasurement laser, OdometryData robotData, std::stack<CheckPoint>& checkpoints)
{
    // Thresholds
    static double thAngle = 30.0;
    // Check the heading angle
    double robotX = robotData.posX;
    double robotY = robotData.posY;
    double checkpointX = checkpoints.top().x;
    double checkpointY = checkpoints.top().y;
    double headingAngle = atan2(checkpointY - robotY, checkpointX - robotX);
    headingAngle = headingAngle * 180.0 / PI;

    // Check the distance where is the robot heading
    double headingDistance = 1234567890;
    for (int i{0}; i < laser.numberOfScans; i++)
    {
        if (laser.Data[i].scanAngle > -1 && laser.Data[i].scanAngle < 1)
        {
            headingDistance = laser.Data[i].scanDistance;
        }
        if (headingDistance < 1000) // There is an obstacle in front of robot
        {
            // Find longest path that is closest to the heading angle
            double longestPath = -1;
            double longestAngle = NULL;
            for (int m{0}; m < laser.numberOfScans; m++)
            {
                if (laser.Data[m].scanDistance > longestPath)
                {
                    longestPath = laser.Data[m].scanDistance;
                    longestAngle = (360.0 - laser.Data[m].scanAngle) * PI / 180.0;
                }
            }
            double newX = robotData.posX + ((longestPath - 500.0) / 1000.0) * cos(longestAngle);
            double newY = robotData.posY - ((longestPath - 500.0) / 1000.0) * sin(longestAngle);
            checkpoints.push({newX, newY});
            std::cout << "X: " << newX << " Y: " << newY << std::endl;
            return;
        }
    }


    // Add some additional space so robot fits and doesn't hit corner
}
