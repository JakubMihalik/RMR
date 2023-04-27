#ifndef TYPES_H
#define TYPES_H

#define ROBOT_RADIUS_MM    350
#define ROBOT_RADIUS_M     0.35
#define ROBOT_DIAMETER_MM (2 * ROBOT_RADIUS_MM)
#define ROBOT_DIAMETER_M  (2 * ROBOT_RADIUS_M)

#define PI 3.1415926536

typedef struct
{
    double x;
    double y;
    double theta;
} ErrorValue;

typedef struct
{
    double x;
    double y;
} Point;

typedef struct
{
  int index;
  double x;
  double y;
  double angle;
  double distance;
  bool isColliding;
} LidarPoint;

inline double degreesToRadians(double degrees)
{
    return (degrees * PI / 180.0);
}

inline double radiansToDegrees(double radians)
{
    return (radians / PI * 180.0);
}

inline Point operator*(const Point& p, double d)
{
    return {p.x * d,  p.y * d};
}

#endif // TYPES_H
