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
        if (laser.Data[i].scanDistance < min && laser.Data[i].scanDistance != 0)
        {
            min = laser.Data[i].scanDistance;
            indx = i;
        }
    }

    DistanceMeasure ret = {laser.Data[indx].scanAngle, laser.Data[indx].scanDistance};
    return ret;
}
