#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

// Depandant includes
#include "rplidar.h"

typedef struct
{
    double angle;
    double distance;
} DistanceMeasure;

class ObjectDetection
{
public:
    ObjectDetection();
    ~ObjectDetection();

public:
    DistanceMeasure readLaserData(LaserMeasurement laser);
};

#endif // OBJECTDETECTION_H
