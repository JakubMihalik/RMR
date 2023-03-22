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
