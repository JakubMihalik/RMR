#include "ObjectDetection.h"

ObjectDetection::ObjectDetection()
{
    // Todo: Constructor
}

ObjectDetection::~ObjectDetection()
{
    // Todo: Destructor
}


DistanceMeasure ObjectDetection::readLaserData(LaserMeasurement laser)
{
    int nScans = laser.numberOfScans;
    double min = 1000000;
    int indx = -1;

    for (int i{0}; i < nScans; i++)
    {
        if (laser.Data[i].scanDistance < min && laser.Data[i].scanDistance >= 0)
        {
            min = laser.Data[i].scanDistance;
            indx = i;
        }
    }

    DistanceMeasure ret = {laser.Data[indx].scanAngle, laser.Data[indx].scanDistance};
    return ret;
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
    }
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
