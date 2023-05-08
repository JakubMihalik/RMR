#ifndef BUGALGORITHM_H
#define BUGALGORITHM_H
#include <vector>
#include "Odometry.h"
#include "Controller.h"
#include "rplidar.h"
#include <limits>
#include "PathPlanning.h"
#include <algorithm>
#include "DBScan.h"

#define DEG2RAD(d) (d * 3.1415926536 / 180.0)

enum RobotState
{
    MOVE_TO_GOAL,
    FOLLOW_OBSTACLE
};
typedef struct  {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
} BoundingBox;

class BugAlgorithm
{
    private:
        RobotState robotState;
        Controller* controller;
        OdometryData* odData;
        Robot* robot;
        DBScan* dbScan;
        bool fProcessedData;
        std::vector<Point> lidarDataXY;
        LaserMeasurement _laserData;
        bool obstacleDetection();
        void processRawLidarData(LaserMeasurement laser);
        bool doLineSegmentsIntersect(Point start, Point goal, Point p2, Point q2);
        bool isObstacleInPath();

        double distance(double x1, double y1, double x2, double y2) {
            return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
        }

        double distance(Point p1, Point p2) {
            return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
        }

        bool isBigJump(float distance1, float distance2, float threshold) {
            return std::abs(distance1 - distance2) > threshold;
        }

        // Variables
        double startingFollowDistanceToGoal;
        Point startingFollowPosition;
        Point start;
        Point goal;
        Point reachingPos;
        Point findNextObstaclePoint(const Point& currentPos, const Point& goal, const std::vector<Point>& obstacles, double maxGoalDistanceIncrease);
        std::vector<Point> obstacles;
        size_t obstacleIndex;
        double threshold = 0.1;
        std::vector<size_t> cornerIndices;



        constexpr static const double SAFE_DISTANCE = 0.5   ; // 30 cm
        constexpr static const double SAFE_ANGLE = DEG2RAD(30); // 30 cm
    public:
        BugAlgorithm(Controller*, OdometryData*, Robot*);

        bool fLidar;
        void setLaserData(LaserMeasurement laserData) { this->_laserData = laserData; this->processRawLidarData(laserData); detectCorners();};
        void followObstacle();

        void controlRobot();
        void stopRobot();
        void turnRobot();
        void detectCorners();

};

#endif // BUGALGORITHM_H
